#pragma once

#include <hubero_ros/task_request_ros_api.h>

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace hubero {

/**
 * @brief Tracks the distance between the actor and another object
 */
class DistanceTracker {
public:
	// How often (at most) the log with a distance update will be shown (in seconds)
	const double PERIODIC_LOG_TIME = 1.0;

	/**
	 * @brief Constructor
	 *
	 * The user should safely join the thread started in the ctor by calling "getThread().join()"
	 *
	 * @param actor a reference HuBeRo actor, a distance between the actor and another object is tracked
	 * @param object_odom_topic name of the topic with the odometry data of another object
	 * @param distance_threshold
	 * @param fun_on_violation function to be executed once the distance threshold is violated
	 */
	explicit DistanceTracker(
		const TaskRequestRosApi& actor,
		const std::string& object_odom_topic,
		const double& distance_threshold,
		std::function<void()>& fun_on_violation
	);

	std::thread getThread() {
		return std::move(t_);
	}

protected:
	std::thread t_;
};

} // namespace hubero
