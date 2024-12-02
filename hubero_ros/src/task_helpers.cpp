#include <hubero_ros/task_helpers.h>

#include <nav_msgs/Odometry.h>

namespace hubero {

DistanceTracker::DistanceTracker(
	const TaskRequestRosApi& actor,
	const std::string& object_odom_topic,
	const double& distance_threshold,
	std::function<void()>& fun_on_violation
) {
	// capture by value only for the DistanceTracker::PERIODIC_LOG_TIME
	auto tracker_odom_cb = [=](
		const nav_msgs::Odometry::ConstPtr& odom,
		const TaskRequestRosApi& actor,
		const double& robot_actor_distance_threshold,
		std::atomic<bool>& distance_threshold_violated,
		const std::function<void()>& fun_on_violation,
		ros::Time& last_log_time
	) -> void {
		if (distance_threshold_violated) {
			// nothing more to do
			return;
		}

		// obtain other's pose
		double x_other = odom->pose.pose.position.x;
		double y_other = odom->pose.pose.position.y;
		// retrieve actor pose in the other frame
		auto target_frame = odom->header.frame_id;
		// get rid of the "/" at the start of the frame name (forbidden in tf2)
		if (!target_frame.empty() && target_frame.at(0) == '/') {
			target_frame.erase(target_frame.begin());
		}
		double x_actor = actor.getPose(target_frame).Pos().X();
		double y_actor = actor.getPose(target_frame).Pos().Y();
		// compute distance between objects
		double dist = std::hypot(x_other - x_actor, y_other - y_actor);

		// logging
		auto time_current = ros::Time::now();
		if ((time_current - last_log_time).toSec() >= DistanceTracker::PERIODIC_LOG_TIME) {
			HUBERO_LOG(
				"[%s].[DistanceTracker] The current distance is %6.3f m (threshold %6.3f m). "
				"The position of the actor is {x: %6.3f, y: %6.3f} and the other object's {x: %6.3f, y: %6.3f} "
				"(in '%s' frame)\r\n",
				actor.getName().c_str(),
				dist,
				distance_threshold,
				x_actor,
				y_actor,
				x_other,
				y_other,
				target_frame.c_str()
			);
			last_log_time = time_current;
		}

		// if distance is small enough, call the function
		if (dist < robot_actor_distance_threshold) {
			distance_threshold_violated = true;
			if (fun_on_violation) {
				HUBERO_LOG(
					"[%s].[DistanceTracker] The threshold distance (%6.3f m) has been violated (%6.3f m currently). "
					"Executing the requested action!\r\n",
					actor.getName().c_str(),
					distance_threshold,
					dist
				);
				fun_on_violation();
			}
		}
	};

	t_ = std::thread(
		[&, tracker_odom_cb, object_odom_topic]() -> void {
			// subscribe robot's movement feedback to trigger the start of movement
			ros::NodeHandle nh("~");
			// indicates end of the tracking
			std::atomic<bool> distance_threshold_violated;
			distance_threshold_violated = false;
			// helper for periodic logging
			auto log_time_last = ros::Time::now();
			// in a subscription callback, distance between objects will be updated
			ros::Subscriber mb_sub = nh.subscribe<nav_msgs::Odometry>(
				object_odom_topic,
				1,
				std::bind(
					std::ref(tracker_odom_cb),
					std::placeholders::_1,
					std::cref(actor),
					std::cref(distance_threshold),
					std::ref(distance_threshold_violated),
					std::ref(fun_on_violation),
					std::ref(log_time_last)
				)
			);

			while (ros::ok() && !distance_threshold_violated) {
				ros::spinOnce();
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}
	);

	HUBERO_LOG(
		"[%s].[DistanceTracker] Started measuring distance against the odometry topic '%s' "
		"with a distance threshold of %.4f m\r\n",
		actor.getName().c_str(),
		object_odom_topic.c_str(),
		distance_threshold
	);
}

} // namespace hubero
