#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_common/typedefs.h>

#include <string>
#include <tuple>

namespace hubero {

/**
 * @brief Creates interface class for algorithms that may provide navigation skills to the HuBeRo actors
 */
class NavigationBase {
public:
	const double GOAL_REACHED_TOLERANCE_DEFAULT = 0.1;

	/**
	 * @brief Default constructor
	 */
	NavigationBase(): initialized_(false), feedback_(TaskFeedbackType::TASK_FEEDBACK_UNDEFINED) {}

	/**
	 * @brief Initialization
	 */
	inline virtual bool initialize(const std::string& actor_name, const std::string& world_frame_name) {
		actor_name_ = actor_name;
		frame_world_ = world_frame_name;
		initialized_ = true;
	}

	inline virtual bool initialize(
		const std::string& actor_name,
		const std::string& world_frame_name,
		const std::string& global_ref_frame_name
	) {
		frame_global_ref_ = global_ref_frame_name;
		initialize(actor_name, world_frame_name);
	}

	/**
	 * @brief Evaluates whether given @ref goal is achievable starting from @ref start
	 *
	 * @note could be const, but ROS version uses non-const serviceclient::call inside
	 */
	inline virtual bool isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) {
		return false;
	}

	/**
	 * @brief Related to localisation
	 *
	 * @note @ref pose must be expressed in the same frame as goal pose in @setGoal
	 * @note Basic implementation mainly for unit testing purposes
	 */
	virtual void update(const Pose3& pose, const Vector3& vel_lin = Vector3(), const Vector3& vel_ang = Vector3()) {
		current_pose_ = pose;
		Vector3 goal_xy(goal_pose_.Pos().X(), goal_pose_.Pos().Y(), 0.0);
		Vector3 curr_xy(current_pose_.Pos().X(), current_pose_.Pos().Y(), 0.0);
		double dist_to_goal = (curr_xy - goal_xy).Length();
		if (dist_to_goal <= NavigationBase::GOAL_REACHED_TOLERANCE_DEFAULT) {
			feedback_ = TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
		} else if (getFeedback() == TaskFeedbackType::TASK_FEEDBACK_PENDING) {
			feedback_ = TaskFeedbackType::TASK_FEEDBACK_ACTIVE;
		}
	}

	/**
	 * @brief Related to simplest navigation task (moving to goal pose)
	 */
	virtual bool setGoal(const Pose3& pose, const std::string& frame) {
		goal_pose_ = pose;
		goal_frame_ = frame;
		feedback_ = TaskFeedbackType::TASK_FEEDBACK_PENDING;
		return true;
	}

	/**
	 * @brief Tries to abort the movement goal, returns true if successful
	 */
	virtual bool cancelGoal() {
		feedback_ = TaskFeedbackType::TASK_FEEDBACK_ABORTED;
		return false;
	}

	/**
	 * @brief Sets feedback to TERMINATED
	 *
	 * @details It is wise to call it once last goal was reached, otherwise it will keep succeeded state
	 */
	virtual void finish() {
		feedback_ = TaskFeedbackType::TASK_FEEDBACK_TERMINATED;
	}

	/**
	 * @brief Computes reachable pose that is closest to the given pose, starting from current pose from update call
	 */
	virtual std::tuple<bool, Pose3> computeClosestAchievablePose(const Pose3& pose, const std::string& frame) {
		return std::make_tuple(false, pose);
	}

	/**
	 * @brief Randomly chooses a reachable goal
	 * @details Goal is expressed in global reference frame, see @ref getGlobalReferenceFrame
	 * @return Tuple: bool is true if goal is valid, Pose3 is reachable pose
	 */
	virtual std::tuple<bool, Pose3> findRandomReachableGoal() {
		return std::make_tuple(false, Pose3());
	}

	/**
	 * @brief Returns TaskFeedbackType
	 */
	inline TaskFeedbackType getFeedback() const {
		return feedback_;
	}

	/**
	 * @brief Returns true if class was initialized successfully
	 */
	inline bool isInitialized() const {
		return initialized_;
	}

	/**
	 * @brief Returns newest velocity command
	 */
	inline virtual Vector3 getVelocityCmd() const {
		return Vector3();
	}

	/**
	 * @brief Retrieves newest goal pose
	 */
	inline virtual Pose3 getGoalPose() const {
		return goal_pose_;
	}

	/**
	 * @brief Retrieves newest goal's frame ID
	 */
	inline virtual std::string getGoalFrame() const {
		return goal_frame_;
	}

	/**
	 * @brief Retrieves name of the world (simulator) frame
	 */
	inline virtual std::string getWorldFrame() const {
		return frame_world_;
	}

	/**
	 * @brief Retrieves name of the global reference (map) frame
	 */
	inline virtual std::string getGlobalReferenceFrame() const {
		return frame_global_ref_;
	}

	/**
	 * @brief Retrieves how far from the goal the actor can be located to accept the global plan
	 */
	inline virtual double getGoalTolerance() const {
		return 1e-03;
	}

	/**
	 * @brief Transforms local velocity (typically received as velocity command) to a global coordinate system
	 */
	static Vector3 convertCommandToGlobalCs(const double& yaw_actor, const Vector3& cmd_vel_local) {
		// Ref: 19-local-to-global-matrix-form at
		// https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
		ignition::math::Matrix3d r(
			+std::cos(yaw_actor), -std::sin(yaw_actor), +0.0,
			+std::sin(yaw_actor), +std::cos(yaw_actor), +0.0,
			+0.0,                 +0.0,                 +1.0
		);
		return r * cmd_vel_local;
	}

	/**
	 * @brief Transforms a global velocity to a local/base coordinate system
	 */
	static Vector3 convertCommandToLocalCs(const double& yaw_actor, const Vector3& vel_global) {
		// Ref: last equation in https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
		ignition::math::Matrix3d r(
			+std::cos(yaw_actor), +std::sin(yaw_actor), +0.0,
			-std::sin(yaw_actor), +std::cos(yaw_actor), +0.0,
			+0.0,                 +0.0,                 +1.0
		);
		return r * vel_global;
	}

protected:
	/// @brief Stores initialization indicator flag
	bool initialized_;

	/// @brief Name of the actor
	std::string actor_name_;

	/// @brief Navigation task feedback
	TaskFeedbackType feedback_;

	/// @brief Name of the frame that incoming ( @ref update ) poses are referenced in
	std::string frame_world_;

	/// @brief Name of the frame that goals chosen with @ref findRandomReachableGoal are referenced in
	std::string frame_global_ref_;

	/// @brief Stores most recent pose from localisation
	Pose3 current_pose_;

	/// @brief Stores newest goal pose
	Pose3 goal_pose_;

	/// @brief Stores newest goal pose frame
	std::string goal_frame_;
}; // class NavigationBase

} // namespace hubero
