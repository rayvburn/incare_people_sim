#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <hubero_common/defines.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_ros/node.h>

#include <string>
#include <cinttypes>
#include <mutex>

namespace hubero {

/**
 * @brief Custom Action client aims to extend standard one with feedback information from action topic
 *
 * @details A specific action client aims to allow end-user to request a specific task from actor agent
 *
 * @tparam Taction type of the action
 * @tparam Tfeedback type of the feedback message
 * @tparam Tresult type of the result message
 */
template <typename Taction, typename Tfeedback, typename Tresult>
class ActionClient: public actionlib::SimpleActionClient<Taction> {
public:
	// TODO: would look cleaner with C++17 'static constexpr'
	/// Defines default size of the queue used for subscriber
	const int SUBSCRIBER_QUEUE_SIZE = 10;

	/**
	 * @brief Constructor of the Action Client
	 *
	 * @param node_ptr pointer to the @ref Node
	 * @param action_task_ns namespace of the action-related ROS topics; should contain action name after last slash
	 */
	ActionClient(
		std::shared_ptr<Node> node_ptr,
		const std::string& action_task_ns
	):
		actionlib::SimpleActionClient<Taction>(action_task_ns, true)
	{
		// action status subscribers
		sub_feedback_ = node_ptr->getNodeHandlePtr()->subscribe(
			action_task_ns + "/" + "feedback",
			SUBSCRIBER_QUEUE_SIZE,
			&ActionClient::callbackFeedback, this
		);
		sub_result_ = node_ptr->getNodeHandlePtr()->subscribe(
			action_task_ns + "/" + "result",
			SUBSCRIBER_QUEUE_SIZE,
			&ActionClient::callbackResult, this
		);
		feedback_status_ = TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;
		action_task_ns_ = action_task_ns;

		if (!ros::ok()) {
			ROS_ERROR("%s: Action server has not been started correctly (ROS is not running)", action_task_ns.c_str());
			return;
		}

		// Wait for server to become online
		while (!this->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("%s: Waiting for the action server to come up: ", action_task_ns.c_str());
			// check if waitingForServer has been terminated by the process finish
			if (!ros::ok()) {
				ROS_ERROR("%s: Action server has not been started correctly", action_task_ns.c_str());
				return;
			}
		}

		ROS_INFO("%s: Action server started!", action_task_ns.c_str());
	}

	/**
	 * @defgroup feedback Feedback topics getters
	 *
	 * @note Types are hard-coded in the action definition file
	 * @{
	 */
	inline TaskFeedbackType getFeedbackStatus() const {
		std::lock_guard<std::mutex> lock(callback_mutex_);
		return feedback_status_;
	}

	inline std::string getFeedbackText() const {
		std::lock_guard<std::mutex> lock(callback_mutex_);
		return feedback_txt_;
	}
	/// @}

protected:
	void callbackFeedback(const Tfeedback& msg) {
		std::lock_guard<std::mutex> lock(callback_mutex_);

		// copy to detect any changes
		TaskFeedbackType status_prev = feedback_status_;
		std::string txt_prev = feedback_txt_;

		feedback_txt_ = std::string(msg->feedback.feedback.text);
		feedback_status_ = static_cast<TaskFeedbackType>(msg->feedback.feedback.status);

		if (feedback_txt_ != txt_prev || feedback_status_ != status_prev) {
			HUBERO_LOG(
				"[ActionClient][%s] Action feedback changed to: {status `%d`, text `%s`}\r\n",
				action_task_ns_.c_str(),
				feedback_status_,
				feedback_txt_.c_str()
			);
		}
	}

	void callbackResult(const Tresult& msg) {
		std::lock_guard<std::mutex> lock(callback_mutex_);

		feedback_txt_ = std::string(msg->result.result.text);
		feedback_status_ = static_cast<TaskFeedbackType>(msg->result.result.status);

		HUBERO_LOG(
			"[ActionClient][%s] Received action result: {status `%d`, text `%s`}\r\n",
			action_task_ns_.c_str(),
			feedback_status_,
			feedback_txt_.c_str()
		);
	}

	std::string action_task_ns_;
	ros::Subscriber sub_feedback_;
	ros::Subscriber sub_result_;
	TaskFeedbackType feedback_status_;
	std::string feedback_txt_;
	mutable std::mutex callback_mutex_;
}; // class ActionClient
} // namespace hubero
