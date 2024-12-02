#include <hubero_ros/navigation_ros.h>

#include <hubero_ros/utils/converter.h>
#include <hubero_ros/utils/misc.h>
#include <hubero_common/logger.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <random>
#include <thread>

namespace hubero {

const int NavigationRos::SUBSCRIBER_QUEUE_SIZE = 10;
const int NavigationRos::PUBLISHER_QUEUE_SIZE = 15;
const int NavigationRos::QUATERNION_RANDOM_RETRY_NUM = 10;
const double NavigationRos::ACTION_RESULT_FREEZE_TIME_SEC = 2.0;
const double NavigationRos::CMD_VEL_KEEP_DURATION = 1.0;

NavigationRos::NavigationRos():
	NavigationBase::NavigationBase(),
	nav_action_server_connected_(false),
	nav_srv_mb_get_plan_exists_(false),
	map_x_min_(0.0),
	map_x_max_(0.0),
	map_y_min_(0.0),
	map_y_max_(0.0),
	tf_listener_(tf_buffer_),
	cmd_vel_timestamp_(ros::Time(0)),
	cmd_vel_timeout_log_timestamp_(ros::Time(0)) {}

bool NavigationRos::initialize(
	std::shared_ptr<Node> node_ptr,
	const std::string& actor_name,
	const std::string& world_frame_name,
	const Pose3& pose_initial
) {
	if (this->isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Already initialized, aborting\r\n", actor_name.c_str());
		return false;
	}

	// initialize base class after objects from args are copied
	pose_initial_ = pose_initial;
	NavigationBase::initialize(actor_name, world_frame_name);

	// helps loading params (looks from a global scope)
	ros::NodeHandle nh;

	// frames
	std::string frame_base_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/actor_frames/base", frame_base_param);
	nh.param(frame_base_param, frame_base_, std::string("base_footprint"));

	std::string frame_global_ref_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/actor_frames/global_ref", frame_global_ref_param);
	nh.param(frame_global_ref_param, frame_global_ref_, std::string("map"));

	std::string frame_local_ref_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/actor_frames/local_ref", frame_local_ref_param);
	nh.param(frame_local_ref_param, frame_local_ref_, std::string("odom"));

	std::string frame_laser_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/actor_frames/lidar", frame_laser_param);
	nh.param(frame_laser_param, frame_laser_, std::string("base_laser_link"));

	std::string frame_camera_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/actor_frames/camera", frame_camera_param);
	nh.param(frame_camera_param, frame_camera_, std::string("base_camera_link"));

	// nav parameters
	std::string param_map_bounds;
	std::vector<double> map_bounds;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/map_bounds", param_map_bounds);
	if (!nh.param(param_map_bounds, map_bounds, std::vector<double>{-1.0, 1.0, -1.0, +1.0})) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not read map bounds from Parameter Server (key: '%s')\r\n",
			actor_name.c_str(),
			param_map_bounds.c_str()
		);
	}
	map_x_min_ = map_bounds.at(0);
	map_x_max_ = map_bounds.at(1);
	map_y_min_ = map_bounds.at(2);
	map_y_max_ = map_bounds.at(3);

	// nav topics
	std::string srv_nav_get_plan;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/get_plan_srv", srv_nav_get_plan);
	nh.param(srv_nav_get_plan, srv_nav_get_plan, std::string("nav/get_plan"));

	std::string topic_nav_cmd;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/command_topic", topic_nav_cmd);
	nh.param(topic_nav_cmd, topic_nav_cmd, std::string("nav/cmd_vel"));

	std::string topic_nav_odometry;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/odometry_topic", topic_nav_odometry);
	nh.param(topic_nav_odometry, topic_nav_odometry, std::string("nav/odom"));

	std::string topic_nav_feedback;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/feedback_topic", topic_nav_feedback);
	nh.param(topic_nav_feedback, topic_nav_feedback, std::string("nav/feedback"));

	std::string topic_nav_result;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/result_topic", topic_nav_result);
	nh.param(topic_nav_result, topic_nav_result, std::string("nav/result"));

	// nav config
	std::string param_nav_get_plan_tolerance;
	nh.searchParam("/hubero_ros/" + actor_name + "/navigation/nav_get_plan_tolerance", param_nav_get_plan_tolerance);
	nh.param(param_nav_get_plan_tolerance, nav_get_plan_tolerance_, 1.0);

	// initialize publishers, service clients, subscribers
	srv_mb_get_plan_ = node_ptr->getNodeHandlePtr()->serviceClient<nav_msgs::GetPlan>(
		srv_nav_get_plan
	);

	sub_cmd_vel_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_cmd,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackCmdVel,
		this
	);

	pub_odom_ = node_ptr->getNodeHandlePtr()->advertise<nav_msgs::Odometry>(
		topic_nav_odometry,
		PUBLISHER_QUEUE_SIZE
	);

	sub_feedback_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_feedback,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackFeedback,
		this
	);

	sub_result_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_result,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackResult,
		this
	);

	if (!srv_mb_get_plan_.isValid()) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Navigation stack '%s' service is not valid\r\n",
			actor_name.c_str(),
			srv_nav_get_plan.c_str()
		);
	}

	HUBERO_LOG("[%s].[NavigationRos] Initialized ROS navigation stack interface\r\n"
		"\t(srv) get plan     client at '%s'\r\n"
		"\t(sub) cmd vel      topic  at '%s'\r\n"
		"\t(sub) feedback     topic  at '%s'\r\n"
		"\t(sub) result       topic  at '%s'\r\n",
		actor_name.c_str(),
		srv_nav_get_plan.c_str(),
		topic_nav_cmd.c_str(),
		topic_nav_feedback.c_str(),
		topic_nav_result.c_str()
	);
	return true;
}

bool NavigationRos::isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response resp;
	req.start.header.frame_id = frame;
	req.start.header.stamp = ros::Time::now();
	req.start.pose = ignPoseToMsgPose(start);
	req.goal.header = req.start.header;
	req.goal.pose = ignPoseToMsgPose(goal);
	req.tolerance = nav_get_plan_tolerance_;
	bool success = srv_mb_get_plan_.call(req, resp);

	if (!success || resp.plan.poses.size() == 0) {
		return false;
	}
	// // TODO: verify
	return true;
}

void NavigationRos::update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return;
	}

	// action server connection - extensive conditions to print this only once
	if (
		nav_action_client_ptr_ != nullptr
		&& !nav_action_server_connected_
		&& nav_action_client_ptr_->isServerConnected()
	) {
		HUBERO_LOG("[%s].[NavigationRos] Connected to ROS navigation action server\r\n", actor_name_.c_str());
		nav_action_server_connected_ = true;
	}

	// service server connection
	if (!nav_srv_mb_get_plan_exists_ && srv_mb_get_plan_.exists()) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Navigation stack '%s' service finally exists\r\n",
			actor_name_.c_str(),
			srv_mb_get_plan_.getService().c_str()
		);
		nav_srv_mb_get_plan_exists_ = true;
	}

	// do not call base class update - let Navigation stack take care about feedback update and goal reaching
	current_pose_ = pose;

	// publish odom
	nav_msgs::Odometry odometry {};
	// header
	odometry.header.stamp = ros::Time::now();
	odometry.header.frame_id = frame_local_ref_;
	odometry.child_frame_id = frame_base_;
	// pose
	Pose3 odom_pose = pose - pose_initial_;
	odometry.pose.pose = ignPoseToMsgPose(odom_pose);
	setIdealCovariance(odometry.pose.covariance);
	// twist - velocities expressed in the base frame -> convert global velocity to the local/base velocity
	Vector3 vel_lin_base = NavigationBase::convertCommandToLocalCs(pose.Rot().Yaw(), vel_lin);
	// assume that Z axis direction matches simulator frame and 'base' frame (typically valid)
	Vector3 vel_ang_base(0.0, 0.0, vel_ang.Z());
	odometry.twist.twist = ignVectorsToMsgTwist(vel_lin_base, vel_ang_base);
	setIdealCovariance(odometry.twist.covariance);
	pub_odom_.publish(odometry);

	/*
	 * Find TFs: world->map, map->odom to publish odometry etc.
	 * NOTE: this is a hack, compared to a typical approach with a real robot, but poses from simulator are expressed
	 * in a global, static frame, whereas we want to create a tree of transforms for the actor - odom->base_footprint.
	 * These computations aim to create a separate TF tree for each actor. All actor trees derive from the one common
	 * frame (typically - "world").
	 */
	bool transform_valid = false;
	Pose3 global_ref_shift;
	std::tie(transform_valid, global_ref_shift) = findTransform(getWorldFrame(), getGlobalReferenceFrame());

	// publish odom TF
	geometry_msgs::TransformStamped transform_odom {};
	transform_odom.header.stamp = ros::Time::now();
	transform_odom.header.frame_id = getGlobalReferenceFrame();
	transform_odom.child_frame_id = frame_local_ref_;
	// ignore height of the pose in this step - this will produce odometry frame at the same height as global ref frame
	Pose3 pose_initial_plane(Vector3(pose_initial_.Pos().X(), pose_initial_.Pos().Y(), 0.0), pose_initial_.Rot());
	transform_odom.transform = ignPoseToMsgTf(pose_initial_plane + global_ref_shift);
	tf_broadcaster_.sendTransform(transform_odom);

	// publish actor base TF
	geometry_msgs::TransformStamped transform_actor {};
	transform_actor.header.stamp = ros::Time::now();
	transform_actor.header.frame_id = frame_local_ref_;
	transform_actor.child_frame_id = frame_base_;
	// elevate with pose_initial_.Pos().Z() that was ignored in the previous TF
	Pose3 pose_base(pose - pose_initial_);
	pose_base.Pos().Z() += pose_initial_.Pos().Z();
	transform_actor.transform = ignPoseToMsgTf(pose_base);
	tf_broadcaster_.sendTransform(transform_actor);

	/*
	 * It's ugly to start action client here, but it seems that move_base waits for odom msg to be received and then
	 * is ready to start. Trying to start the action client in @ref initialize freezes everything. This was helpful:
	 * https://answers.ros.org/question/345012/move_base-action-topics-exist-but-client-stuck-on-waitforserver/
	 */
	if (nav_action_client_ptr_ == nullptr) {
		// find action client namespace, based on e.g. odom topic
		auto action_ns = pub_odom_.getTopic();
		if (action_ns.back() == '/') {
			action_ns.pop_back();
		}
		auto action_ns_separator = action_ns.find_last_of("/");
		action_ns = action_ns.substr(0, action_ns_separator);

		nav_action_client_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
			action_ns,
			true
		);
	}
}

bool NavigationRos::setGoal(const Pose3& pose, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'setGoal' request\r\n",
			actor_name_.c_str()
		);
		return false;
	}

	auto nav_goal_backup = nav_goal_;

	auto time_current = ros::Time::now();
	actionlib_msgs::GoalID goal_id;
	goal_id.id = "";
	goal_id.stamp = time_current;

	nav_goal_.target_pose.header.frame_id = frame;
	nav_goal_.target_pose.header.stamp = time_current;
	nav_goal_.target_pose.pose = ignPoseToMsgPose(pose);

	// evaluate goal quaternion
	if (!NavigationRos::isQuaternionValid(pose.Rot())) {
		HUBERO_LOG(
			"[%s].[NavigationRos] New goal will not be set - its quaternion is not valid\r\n",
			actor_name_.c_str()
		);
		nav_goal_ = nav_goal_backup;
		return false;
	}

	NavigationBase::setGoal(pose, frame);

	/*
	 * - SimpleDoneCallback: returns action status with a big delay (compared to topic data) that interferes HuBeRo
	 * task-switching logic
	 * - SimpleActiveCallback: we subscribe more extensive action state feedback
	 * - SimpleFeedbackCallback: feedback returns current pose of the 'base', which we know quite well
	 */
	nav_action_client_ptr_->sendGoal(
		nav_goal_,
		MoveBaseActionClient::SimpleDoneCallback(),
		MoveBaseActionClient::SimpleActiveCallback(),
		MoveBaseActionClient::SimpleFeedbackCallback()
	);

	HUBERO_LOG(
		"[%s].[NavigationRos] Set goal: x %1.2f, y %1.2f, z %1.2f, yaw %1.2f° (frame: %s)\r\n",
		actor_name_.c_str(),
		pose.Pos().X(),
		pose.Pos().Y(),
		pose.Pos().Z(),
		IGN_RTOD(pose.Rot().Yaw()),
		frame.c_str()
	);
	return true;
}

bool NavigationRos::cancelGoal() {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'cancelGoal' request\r\n",
			actor_name_.c_str()
		);
		return false;
	}

	NavigationBase::cancelGoal();
	nav_action_client_ptr_->cancelGoal();
	HUBERO_LOG("[%s].[NavigationRos] Trying to cancel all navigation goals\r\n", actor_name_.c_str());
	return true;
}

void NavigationRos::finish() {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return;
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'finish' call\r\n",
			actor_name_.c_str()
		);
		return;
	}

	nav_action_client_ptr_->cancelAllGoals();
	NavigationBase::finish();
}

std::tuple<bool, Pose3> NavigationRos::computeClosestAchievablePose(const Pose3& pose, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return std::make_tuple(false, pose);
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring request\r\n",
			actor_name_.c_str()
		);
		return std::make_tuple(false, pose);
	}

	// compute plan
	auto path = computePlan(current_pose_, getWorldFrame(), pose, frame);

	bool goal_pose_ok = false;
	Pose3 goal_pose_potential;
	std::tie(goal_pose_ok, goal_pose_potential) = selectGoalFromPlan(path);

	if (!goal_pose_ok) {
		HUBERO_LOG("[%s].[NavigationRos] Could not compute pose closest to given pose\r\n", actor_name_.c_str());
	}
	return std::make_tuple(goal_pose_ok, goal_pose_potential);
}

std::tuple<bool, Pose3> NavigationRos::findRandomReachableGoal() {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return std::make_tuple(false, Pose3());
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring request\r\n",
			actor_name_.c_str()
		);
		return std::make_tuple(false, Pose3());
	}

	// https://stackoverflow.com/questions/7560114/
	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_real_distribution<> distr_x(map_x_min_, map_x_max_);
	double x_draw = distr_x(gen);

	std::uniform_real_distribution<> distr_y(map_y_min_, map_y_max_);
	double y_draw = distr_y(gen);

	// draw yaw
	std::uniform_real_distribution<> distr_yaw(-IGN_PI, +IGN_PI);
	double yaw_draw = 0.0;
	bool quaternion_valid = false;

	/*
	 * move_base complains about quaternion:
	 * "Quaternion has length close to zero... discarding as navigation goal"
	 */
	for (int i = 0; i < QUATERNION_RANDOM_RETRY_NUM; i++) {
		yaw_draw = distr_yaw(gen);
		auto q = Quaternion(current_pose_.Rot().Roll(), current_pose_.Rot().Pitch(), yaw_draw);
		quaternion_valid = NavigationRos::isQuaternionValid(q);
		if (quaternion_valid) {
			break;
		}
	}

	// evaluate if the valid quaternion was found - abort further actions if it is not a valid one
	if (!quaternion_valid) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not find a valid quaternion for a randomly chosen goal. "
			"Aborting this attempt\r\n",
			actor_name_.c_str()
		);
		return std::make_tuple(false, Pose3());
	}

	// compose a potentially new goal based on random generator output and current pose
	Pose3 goal_potential(
		x_draw,
		y_draw,
		current_pose_.Pos().Z(),
		current_pose_.Rot().Roll(),
		current_pose_.Rot().Pitch(),
		yaw_draw
	);

	// compute plan
	auto path = computePlan(current_pose_, getWorldFrame(), goal_potential, getGlobalReferenceFrame());

	// choose goal pose from plan
	bool goal_pose_ok = false;
	Pose3 goal_pose_potential;
	std::tie(goal_pose_ok, goal_pose_potential) = selectGoalFromPlan(path);

	if (!goal_pose_ok) {
		HUBERO_LOG("[%s].[NavigationRos] Could not randomly choose a goal\r\n", actor_name_.c_str());
	}
	return std::make_tuple(goal_pose_ok, goal_pose_potential);
}

Vector3 NavigationRos::getVelocityCmd() const {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return Vector3();
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'getVelocityCmd'\r\n",
			actor_name_.c_str()
		);
		return Vector3();
	}

	// evaluate the age of the command
	double command_age = (ros::Time::now() - cmd_vel_timestamp_).toSec();
	if (command_age > NavigationRos::CMD_VEL_KEEP_DURATION) {
		if ((ros::Time::now() - cmd_vel_timeout_log_timestamp_).toSec() >= 1.0) {
			cmd_vel_timeout_log_timestamp_ = ros::Time::now();
			HUBERO_LOG(
				"[%s].[NavigationRos] Publishing zero velocity as the latest command is outdated - it's %.1fs old, "
				"while the timeout is %.1fs\r\n",
				actor_name_.c_str(),
				command_age,
				NavigationRos::CMD_VEL_KEEP_DURATION
			);
		}
		return Vector3();
	}
	return cmd_vel_;
}

// static
bool NavigationRos::isQuaternionValid(const Quaternion& q) {
	// first we need to check if the quaternion has nan's or infs
	if (!std::isfinite(q.X()) || !std::isfinite(q.Y()) || !std::isfinite(q.Z()) || !std::isfinite(q.W())) {
		return false;
	}

	tf2::Quaternion tf_q(q.X(), q.Y(), q.Z(), q.W());

	//next, we need to check if the length of the quaternion is close to zero
	if(tf_q.length2() < 1e-6){
		return false;
	}

	// next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
	tf_q.normalize();

	tf2::Vector3 up(0, 0, 1);

	double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

	if(fabs(dot - 1) > 1e-3){
		return false;
	}

	return true;
}

// static
bool NavigationRos::isMoveBaseBusy(const actionlib::SimpleClientGoalState& state) {
	return state != actionlib::SimpleClientGoalState::LOST
		&& state != actionlib::SimpleClientGoalState::ABORTED
		&& state != actionlib::SimpleClientGoalState::SUCCEEDED
		&& state != actionlib::SimpleClientGoalState::PREEMPTED;
}

void NavigationRos::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	Vector3 cmd_vel_local;
	cmd_vel_local.X(msg->linear.x);
	cmd_vel_local.Y(msg->linear.y);
	cmd_vel_local.Z(msg->angular.z);
	cmd_vel_ = NavigationBase::convertCommandToGlobalCs(current_pose_.Rot().Yaw(), cmd_vel_local);
	cmd_vel_timestamp_ = ros::Time::now();
}

void NavigationRos::callbackFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	auto fb_type = convertActionStatusToTaskFeedback(msg->status.status);
	if (fb_type == TASK_FEEDBACK_UNDEFINED) {
		return;
	}
	// The aim is to prevent a late action feedback overriding the result (stored in @ref feedback_)
	if (
		(ros::Time::now() - cb_result_timestamp_).toSec() <= NavigationRos::ACTION_RESULT_FREEZE_TIME_SEC
		&& fb_type == TASK_FEEDBACK_ACTIVE
	) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Action feedback won't be overwritten because the action result has recently been updated (to %d). "
			"Ignored feedback status is %d\r\n",
			actor_name_.c_str(),
			feedback_,
			fb_type
		);
		return;
	}
	feedback_ = fb_type;
}

void NavigationRos::callbackResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	auto fb_type = convertActionStatusToTaskFeedback(msg->status.status);
	if (fb_type == TASK_FEEDBACK_UNDEFINED) {
		return;
	}
	feedback_ = fb_type;
	// save for later use in @ref callbackFeedback
	cb_result_timestamp_ = ros::Time::now();
}

std::tuple<bool, Pose3> NavigationRos::findTransform(const std::string& frame_source, const std::string& frame_target) const {
	Pose3 transform;
	bool success = false;
	try {
		auto tf_msg = tf_buffer_.lookupTransform(frame_target, frame_source, ros::Time(0));
		transform = msgTfToPose(tf_msg.transform);
		success = true;
	} catch (tf2::TransformException& ex) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not transform '%s' to '%s' - exception: '%s'\r\n",
			actor_name_.c_str(),
			getWorldFrame().c_str(),
			getGlobalReferenceFrame().c_str(),
			ex.what()
		);
	}
	return std::make_tuple(success, transform);
}

nav_msgs::Path NavigationRos::computePlan(
	const Pose3& start_pose,
	const std::string& start_frame,
	const Pose3& goal_pose,
	const std::string& goal_frame
) {
	/*
	 * computePlan in a nutshell:
	 * store backup of the current goal, abort it, compute plan
	 * and restore the previous goal
	 */
	if (nav_action_client_ptr_->getState() == actionlib::SimpleClientGoalState::ACTIVE
		|| nav_action_client_ptr_->getState() == actionlib::SimpleClientGoalState::PENDING
	) {
		// this order allows to abort current goal and not update navigation status (callbacks detached)
		nav_action_client_ptr_->stopTrackingGoal();
		nav_action_client_ptr_->cancelAllGoals();
	}

	// service complains if start/goal pose is defined in frame other than the global reference frame
	bool transform_start_valid = false;
	Pose3 transform_start;
	std::tie(transform_start_valid, transform_start) = findTransform(start_frame, getGlobalReferenceFrame());

	bool transform_goal_valid = false;
	Pose3 transform_goal;
	std::tie(transform_goal_valid, transform_goal) = findTransform(goal_frame, getGlobalReferenceFrame());

	bool transform_world_valid = false;
	Pose3 transform_world;
	std::tie(transform_world_valid, transform_world) = findTransform(getGlobalReferenceFrame(), getWorldFrame());

	if (!transform_start_valid || !transform_goal_valid || !transform_world_valid) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not compute valid plan because of TF lookup failure\r\n",
			actor_name_.c_str()
		);
		return nav_msgs::Path();
	}

	auto pose_start_global_ref = start_pose + transform_start;
	auto pose_goal_global_ref = goal_pose + transform_goal;

	// "planarize" poses
	auto pose_start_global_ref_plane = pose_start_global_ref;
	pose_start_global_ref_plane.Pos().Z(0.0);

	auto pose_goal_global_ref_plane = pose_goal_global_ref;
	pose_goal_global_ref_plane.Pos().Z(0.0);

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response resp;
	auto ts = ros::Time::now();

	req.start.header.frame_id = getGlobalReferenceFrame();
	req.start.header.stamp = ts;
	req.start.header.seq = 0;
	req.start.pose = ignPoseToMsgPose(pose_start_global_ref_plane);

	req.goal.header.frame_id = getGlobalReferenceFrame();
	req.goal.header.stamp = ts;
	req.goal.header.seq = 0;
	req.goal.pose = ignPoseToMsgPose(pose_goal_global_ref_plane);
	req.tolerance = nav_get_plan_tolerance_;

	/*
	 * wait for action client to become free (LOST -> after cancel) to ask for a plan, ROS ERROR:
	 * move_base must be in an inactive state to make a plan for an external user
	 * LOST: when goals are canceled
	 * ABORT: when e.g. goal could not be reached and was aborted
	 */
	while (NavigationRos::isMoveBaseBusy(nav_action_client_ptr_->getState())) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Waiting for navigation action client to compute a plan. Client state: %s\r\n",
			actor_name_.c_str(),
			nav_action_client_ptr_->getState().toString().c_str()
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	/*
	 * // FIXME: process this call in separate thread and set goal according to the generated plan
	 * sometimes 1 sec of delay is not enough to force plan computation;
	 * Experiments show that the plan will be generated in first srv call or during next computePlan call.
	 * Sometimes delay like below helped, but caused massive lags during simulation:
	 *
	 * std::this_thread::sleep_for(std::chrono::milliseconds(1));
	 *
	 */
	bool success = srv_mb_get_plan_.call(req, resp);
	if (!success) {
		return nav_msgs::Path();;
	}

	// restore previous goal if it's a valid one
	auto prev_goal = Pose3(msgPoseToIgnPose(nav_goal_.target_pose.pose));
	if (NavigationRos::isQuaternionValid(prev_goal.Rot())) {
		nav_action_client_ptr_->sendGoal(
			nav_goal_,
			MoveBaseActionClient::SimpleDoneCallback(),
			MoveBaseActionClient::SimpleActiveCallback(),
			MoveBaseActionClient::SimpleFeedbackCallback()
		);
	}

	if (!success || resp.plan.poses.size() == 0) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Couldn't find a plan from {x %2.1f, y %2.1f} to {x %2.1f, y %2.1f} despite tolerance of %2.4f\r\n",
			actor_name_.c_str(),
			start_pose.Pos().X(),
			start_pose.Pos().Y(),
			goal_pose.Pos().X(),
			goal_pose.Pos().Y(),
			nav_get_plan_tolerance_
		);
		return nav_msgs::Path();
	}

	HUBERO_LOG(
		"[%s].[NavigationRos] Computed plan with %lu poses (goal: {%2.2f, %2.2f, %2.2f}, "
		"lastElem: {%2.2f, %2.2f}, secToLastElem: {%2.2f, %2.2f}\r\n",
		actor_name_.c_str(),
		resp.plan.poses.size(),
		pose_goal_global_ref_plane.Pos().X(),
		pose_goal_global_ref_plane.Pos().Y(),
		pose_goal_global_ref_plane.Rot().Yaw(),
		resp.plan.poses.back().pose.position.x,
		resp.plan.poses.back().pose.position.y,
		resp.plan.poses.end()[-2].pose.position.x,
		resp.plan.poses.end()[-2].pose.position.y
	);

	// convert to the world frame, if needed
	if (getGlobalReferenceFrame() != getWorldFrame()) {
		for (auto& pose: resp.plan.poses) {
			Pose3 posehubero = msgPoseToIgnPose(pose.pose);
			posehubero = posehubero + transform_world;
			pose.pose = ignPoseToMsgPose(posehubero);
			pose.header.frame_id = getWorldFrame();
		}
	}
	return resp.plan;
}

std::tuple<bool, Pose3> NavigationRos::selectGoalFromPlan(const nav_msgs::Path& path) {
	// goal pose is not reachable when plan consists of a set of valid poses + goal pose at the back of vector;
	// NOTE: this is parameterized and can be disabled
	if (path.poses.size() >= 2) {
		// second to last element in a vector
		auto pose_goal = msgPoseToIgnPose(path.poses.end()[-2].pose);
		// evaluate quaternion of the new potential goal
		bool goal_quaternion_valid = NavigationRos::isQuaternionValid(pose_goal.Rot());
		if (!goal_quaternion_valid) {
			HUBERO_LOG(
				"[%s].[NavigationRos] Quaternion of the plan final pose is invalid. Aborting this attempt\r\n",
				actor_name_.c_str()
			);
			// no luck finding goal
			return std::make_tuple(false, Pose3());
		}
		// quaternion is valid, let pose_goal be used elsewhere as new navigation goal
		return std::make_tuple(true, pose_goal);

	} else if (path.poses.empty()) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Computed plan is empty - Aborting this attempt\r\n",
			actor_name_.c_str()
		);
		return std::make_tuple(false, Pose3());
	}

	HUBERO_LOG(
		"[%s].[NavigationRos] Computed plan is not empty (%d) but most likely contains something strange\r\n",
		actor_name_.c_str(),
		static_cast<int>(path.poses.size())
	);
	return std::make_tuple(false, Pose3());
}

} // namespace hubero
