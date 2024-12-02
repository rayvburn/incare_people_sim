#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/typedefs.h>

#include <hubero_ros/utils/action_client.h>
#include <hubero_ros/node.h>

#include <hubero_ros_msgs/FollowObjectAction.h>
#include <hubero_ros_msgs/LieDownAction.h>
#include <hubero_ros_msgs/LieDownObjectAction.h>
#include <hubero_ros_msgs/MoveAroundAction.h>
#include <hubero_ros_msgs/MoveToGoalAction.h>
#include <hubero_ros_msgs/MoveToObjectAction.h>
#include <hubero_ros_msgs/RunAction.h>
#include <hubero_ros_msgs/SitDownAction.h>
#include <hubero_ros_msgs/SitDownObjectAction.h>
#include <hubero_ros_msgs/StandAction.h>
#include <hubero_ros_msgs/TalkAction.h>
#include <hubero_ros_msgs/TalkObjectAction.h>
#include <hubero_ros_msgs/TeleopAction.h>

#include <tf2_ros/transform_listener.h>

#include <memory>
#include <thread>

namespace hubero {

/**
 * @brief Class that stands for a client for @ref TaskRequestRos class
 *
 * @details This class can be directly used to request tasks via ROS Action Servers as long as @ref TaskRequestRos
 * interface for a specific actor is operational. @ref TaskRequestRosApi wraps actionlib client and is intended
 * to be allocated in C++ scenario executables with HuBeRo actors.
 */
class TaskRequestRosApi {
public:
	/// Defines the duration of sleeps between subsequent processings of the callback queue
	const int CALLBACK_SPINNING_SLEEP_TIME_MS = 10;

	/// Defines the duration of sleeps between subsequent evaluations of task conditions
	const int THREADED_EXECUTOR_SLEEP_TIME_MS = 100;

	/**
	 * @brief Constructor of @ref TaskRequestRosApi instance
	 * @param actor_name actor identifier from simulation
	 */
	TaskRequestRosApi(const std::string& actor_name);

	/**
	 * @brief Destructor that kills all spawned threads
	 */
	virtual ~TaskRequestRosApi();

	/**
	 * @defgroup tasks Methods that allow to request specific task from controlled actor
	 * @{
	 */
	/**
	 * @defgroup followobject Operations related to follow object task
	 */
	/**
	 * @brief Requests actor to follow object that in simulation is recognized by @ref object_name
	 */
	bool followObject(const std::string& object_name);

	/**
	 * @brief Requests actor to follow object according to @ref goal
	 */
	bool followObject(const hubero_ros_msgs::FollowObjectGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopFollowingObject();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getFollowObjectState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getFollowObjectStateDescription() const;

	/** @} */ // end of followobject group

	/**
	 * @defgroup liedown Operations related to lie down task
	 */
	/**
	 * @brief Requests actor to lie down at @ref pos point with orientation @ref yaw
	 * @details @ref pos frame (coordinate system) of reference is given by @ref frame_id
	 */
	bool lieDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to lie down according to @ref goal
	 */
	bool lieDown(const hubero_ros_msgs::LieDownGoal& goal);

	/**
	 * @brief Requests actor to lie down onto @ref object_name object with orientation @ref yaw at @ref height height
	 */
	bool lieDownObject(const std::string& object_name, const double& height, const double& yaw);

	/**
	 * @brief Requests actor to lie down onto object according to @ref goal
	 */
	bool lieDownObject(const hubero_ros_msgs::LieDownObjectGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopLyingDown();
	bool stopLyingDownObject();

	/**
	 * @brief Returns most recent state of the task
	 *
	 * @note Currently: preempted -> finished
	 */
	TaskFeedbackType getLieDownState() const;
	TaskFeedbackType getLieDownObjectState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getLieDownStateDescription() const;
	std::string getLieDownObjectStateDescription() const;

	/** @} */ // end of liedown group

	/**
	 * @defgroup movearound Operations related to move around task
	 */
	/**
	 * @brief Requests actor to move around (choose his navigation goals randomly)
	 */
	bool moveAround();

	/**
	 * @brief Requests actor to move around (choose his navigation goals randomly)
	 */
	bool moveAround(const hubero_ros_msgs::MoveAroundGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopMovingAround();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getMoveAroundState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getMoveAroundStateDescription() const;

	/** @} */ // end of movearound group

	/**
	 * @defgroup movetogoal Operations related to move to goal task
	 */
	/**
	 * @brief Requests actor to move to @ref pos point, where @ref pos frame of reference is given by @ref frame_id
	 */
	bool moveToGoal(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to move to goal according to @ref goal
	 */
	bool moveToGoal(const hubero_ros_msgs::MoveToGoalGoal& goal);

	/**
	 * @brief Requests actor to move to specific object named @ref object_name
	 */
	bool moveToObject(const std::string& object_name);

	/**
	 * @brief Requests actor to move to specific object according to @ref goal
	 */
	bool moveToObject(const hubero_ros_msgs::MoveToObjectGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopMovingToGoal();
	bool stopMovingToObject();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getMoveToGoalState() const;
	TaskFeedbackType getMoveToObjectState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getMoveToGoalStateDescription() const;
	std::string getMoveToObjectStateDescription() const;

	/**
	 * @brief Implements a typical use case where an actor is intended to travel through a given set of waypoints
	 *
	 * Execution state can be checked with @ref isThreadExecuting
	 * This method mustn't be used once another task has been started with the @ref startThreadedExecution
	 *
	 * The user must provide that the returned thread object won't go out of scope until the thread is finished.
	 * Additionally, they should call the std::thread.join() at the end of the scenario executable.
	 */
	std::thread moveThroughWaypoints(
		const std::vector<std::pair<Vector3, double>>& poses2d,
		const std::string& frame,
		const ros::Duration& timeout = ros::Duration(0.0)
	);

	/** @} */ // end of movetogoal group

	/**
	 * @defgroup run Operations related to run task
	 */
	/**
	 * @brief Requests actor to run to @ref pos point, where @ref pos frame of reference is given by @ref frame_id
	 */
	bool run(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to run to goal point according to @ref goal
	 */
	bool run(const hubero_ros_msgs::RunGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopRunning();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getRunState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getRunStateDescription() const;

	/** @} */ // end of run group

	/**
	 * @defgroup sitdown Operations related to sit down task
	 */
	/**
	 * @brief Requests actor to sit down at @ref pos point with orientation @ref yaw
	 * @details @ref pos frame (coordinate system) of reference is given by @ref frame_id
	 */
	bool sitDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to sit down according to @ref goal
	 */
	bool sitDown(const hubero_ros_msgs::SitDownGoal& goal);

	/**
	 * @brief Requests actor to sit down onto @ref object_name object with orientation @ref yaw at @ref height height
	 */
	bool sitDownObject(const std::string& object_name, const double& height, const double& yaw);

	/**
	 * @brief Requests actor to sit down onto object according to @ref goal
	 */
	bool sitDownObject(const hubero_ros_msgs::SitDownObjectGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopSittingDown();
	bool stopSittingDownObject();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getSitDownState() const;
	TaskFeedbackType getSitDownObjectState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getSitDownStateDescription() const;
	std::string getSitDownObjectStateDescription() const;

	/** @} */ // end of sitdown group

	/**
	 * @defgroup stand Operations related to stand task
	 */
	/**
	 * @brief Requests actor to stand still
	 */
	bool stand();

	/**
	 * @brief Requests actor to stand still according to @ref goal
	 * @details Stand cannot be stopped, it is treated as 'idle' state
	 */
	bool stand(const hubero_ros_msgs::StandGoal& goal);

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getStandState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getStandStateDescription() const;

	/** @} */ // end of stand group

	/**
	 * @defgroup talk Operations related to talk task
	 */
	/**
	 * @brief Requests actor to talk, but first let it move to @ref pos position expressed in @ref frame_id frame
	 */
	bool talk(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to talk according to @ref goal
	 */
	bool talk(const hubero_ros_msgs::TalkGoal& goal);

	/**
	 * @brief Requests actor to talk to object (move towards that object and start talking) given by @ref object_name
	 */
	bool talkObject(const std::string& object_name);

	/**
	 * @brief Requests actor to talk to object according to @ref goal
	 * @details Also @see talkObject
	 */
	bool talkObject(const hubero_ros_msgs::TalkObjectGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopTalking();
	bool stopTalkingObject();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getTalkState() const;
	TaskFeedbackType getTalkObjectState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getTalkStateDescription() const;
	std::string getTalkObjectStateDescription() const;

	/** @} */ // end of talk group

	/**
	 * @defgroup talk Operations related to talk task
	 */
	/**
	 * @brief Requests actor to teleoperate
	 */
	bool teleop();

	/**
	 * @brief Requests actor to teleoperate according to @ref goal
	 */
	bool teleop(const hubero_ros_msgs::TeleopGoal& goal);

	/**
	 * @brief Aborts task execution
	 */
	bool stopTeleop();

	/**
	 * @brief Returns most recent state of the task
	 */
	TaskFeedbackType getTeleopState() const;

	/**
	 * @brief Returns most recent state of the task expressed as string
	 */
	std::string getTeleopStateDescription() const;

	/** @} */ // end of talk group
	/** @} */ // end of tasks group

	/**
	 * @brief Returns name of the actor that task request API was created for
	 */
	inline std::string getName() const {
		return actor_name_;
	}

	/**
	 * @brief Obtains a pose of the actor in a frame given by @ref frame_reference
	 */
	Pose3 getPose(const std::string& frame_reference) const;

	/**
	 * @defgroup execution Methods related to a task execution in a separate thread
	 */
	/**
	 * @brief Starts the execution of a single task defined by the arguments in a separate thread
	 *
	 * This method does not stop previously started task in any way.
	 *
	 * @param state_checker_fun a reference to a function that allows checking the state of task execution,
	 * e.g., @ref getTeleopState; reference is a must here - inline lambda won't work
	 * @param logging_task_name task name used in logs
	 * @param timeout timeout for task execution (by default - no limit)
	 * @param state_executing a state in which the task is assumed to be executing
	 */
	void startThreadedExecution(
		const std::function<TaskFeedbackType()>& state_checker_fun,
		const std::string& logging_task_name,
		const ros::Duration& timeout = ros::Duration(0.0),
		TaskFeedbackType state_executing = TaskFeedbackType::TASK_FEEDBACK_ACTIVE
	);

	/// Evaluates whether the thread that executes the newest task does still perform computations
	inline bool isThreadExecuting() const {
		return threaded_task_execution_requested_ && !threaded_task_execution_done_;
	}

	/**
	 * @brief Joins the thread that executes the newest task
	 *
	 * Can be also treated as a method that executes a task started with @ref startThreadedExecution in a blocking way
	 */
	void join();

	/// @brief Calls @ref std::thread::joinable method on the @ref task_executor_ thread
	inline bool joinable() const {
		return task_executor_.joinable();
	}

	/** @} */ // end of execution group

	/**
	 * @defgroup utils Static functions that may be handy when designing a custom scenario
	 */
	/// Sleeps the current thread for the @ref ms milliseconds
	static void wait(const std::chrono::milliseconds& ms = std::chrono::milliseconds(500));

	/// Sleeps the current thread for the @ref seconds regarding the ROS time
	static void waitRosTime(double seconds);

	/// Sleeps the current thread for the @ref duration regarding the ROS time
	static void waitRosTime(const ros::Duration& duration);

	/** @} */ // end of utils group

protected:
	std::shared_ptr<Node> node_ptr_;

	/// Alias declaration
	template <typename Taction, typename Tfeedback, typename Tresult>
	using ActionClientPtr = std::shared_ptr<ActionClient<Taction, Tfeedback, Tresult>>;

	ActionClientPtr<
		hubero_ros_msgs::FollowObjectAction,
		hubero_ros_msgs::FollowObjectActionFeedbackConstPtr,
		hubero_ros_msgs::FollowObjectActionResultConstPtr> ac_follow_object_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::LieDownAction,
		hubero_ros_msgs::LieDownActionFeedbackConstPtr,
		hubero_ros_msgs::LieDownActionResultConstPtr> ac_lie_down_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::LieDownObjectAction,
		hubero_ros_msgs::LieDownObjectActionFeedbackConstPtr,
		hubero_ros_msgs::LieDownObjectActionResultConstPtr> ac_lie_down_object_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::MoveAroundAction,
		hubero_ros_msgs::MoveAroundActionFeedbackConstPtr,
		hubero_ros_msgs::MoveAroundActionResultConstPtr> ac_move_around_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::MoveToGoalAction,
		hubero_ros_msgs::MoveToGoalActionFeedbackConstPtr,
		hubero_ros_msgs::MoveToGoalActionResultConstPtr> ac_move_to_goal_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::MoveToObjectAction,
		hubero_ros_msgs::MoveToObjectActionFeedbackConstPtr,
		hubero_ros_msgs::MoveToObjectActionResultConstPtr> ac_move_to_object_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::RunAction,
		hubero_ros_msgs::RunActionFeedbackConstPtr,
		hubero_ros_msgs::RunActionResultConstPtr> ac_run_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::SitDownAction,
		hubero_ros_msgs::SitDownActionFeedbackConstPtr,
		hubero_ros_msgs::SitDownActionResultConstPtr> ac_sit_down_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::SitDownObjectAction,
		hubero_ros_msgs::SitDownObjectActionFeedbackConstPtr,
		hubero_ros_msgs::SitDownObjectActionResultConstPtr> ac_sit_down_object_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::StandAction,
		hubero_ros_msgs::StandActionFeedbackConstPtr,
		hubero_ros_msgs::StandActionResultConstPtr> ac_stand_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::TalkAction,
		hubero_ros_msgs::TalkActionFeedbackConstPtr,
		hubero_ros_msgs::TalkActionResultConstPtr> ac_talk_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::TalkObjectAction,
		hubero_ros_msgs::TalkObjectActionFeedbackConstPtr,
		hubero_ros_msgs::TalkObjectActionResultConstPtr> ac_talk_object_ptr_;
	ActionClientPtr<
		hubero_ros_msgs::TeleopAction,
		hubero_ros_msgs::TeleopActionFeedbackConstPtr,
		hubero_ros_msgs::TeleopActionResultConstPtr> ac_teleop_ptr_;

	/**
	 * @brief Sends action goal via given action client
	 */
	template <typename Tacptr, typename Tgoal>
	bool sendActionGoal(Tacptr& action_client_ptr, const Tgoal& action_goal) {
		if (action_client_ptr == nullptr) {
			return false;
		}
		action_client_ptr->sendGoal(action_goal);
		return true;
	}

	/**
	 * @brief Cancels all action goals
	 */
	template <typename Tacptr>
	bool cancelActionGoals(Tacptr& action_client_ptr) {
		if (action_client_ptr == nullptr) {
			return false;
		}
		action_client_ptr->cancelAllGoals();
		return true;
	}

	/**
	 * @brief Queries given action action client for feedback in enum form
	 */
	template <typename Tacptr>
	TaskFeedbackType getActionStateType(Tacptr& action_client_ptr) const {
		if (action_client_ptr == nullptr) {
			return TASK_FEEDBACK_UNDEFINED;
		}
		return action_client_ptr->getFeedbackStatus();
	}

	/**
	 * @brief Queries given action action client for feedback in text form
	 */
	template <typename Tacptr>
	std::string getActionStateDescription(Tacptr& action_client_ptr) const {
		if (action_client_ptr == nullptr) {
			return std::string("Action client is not operational (nullptr)");
		}
		return action_client_ptr->getFeedbackText();
	}

	/**
	 * A method that is executed in a separate thread (@ref callback_spinner_); periodically calls `ros::spinOnce`
	 *
	 * This is necessary for ActionServers to receive messages (otherwise is must've been handled by the user in his
	 * orchestrating node).
	 */
	virtual void callbackProcessingThread();

	/**
	 * Takes care of the execution of a task defined by the arguments.
	 *
	 * This should be called immediately after sending a task request.
	 *
	 * @param state_checker_fun a function that allows checking the state of task execution, e.g., @ref getTeleopState
	 * @param logging_task_name task name used in logs
	 * @param timeout timeout for task execution (by default no limit)
	 * @param state_executing a state in which the task is assumed to be executing
	 */
	void threadedExecutor(
		const std::function<TaskFeedbackType()>& state_checker_fun,
		const std::string& logging_task_name,
		const ros::Duration& timeout,
		TaskFeedbackType state_executing
	);

private:
	/// Name of the actor provided in the constructor
	std::string actor_name_;

	/// A separate thread for processing callbacks
	std::thread callback_spinner_;

	/// A thread for executing a task
	std::thread task_executor_;
	/// Atomic flag indicating the request of a threaded execution of a task
	std::atomic<bool> threaded_task_execution_requested_;
	/// Atomic flag indicating the finish of computations for @ref task_executor_ thread
	std::atomic<bool> threaded_task_execution_done_;

	/// Flag set in the destructor to join threads
	std::atomic<bool> destructing_;

	/// Buffer for obtaining ROS transforms (i.a. self pose)
	tf2_ros::Buffer tf_buffer_;
	/// Listener for obtaining ROS transforms (i.a. self pose)
	tf2_ros::TransformListener tf_listener_;
	/// Name of the TF frame related to the actor
	std::string actor_tf_frame_;
}; // class TaskRequestRosApi
} // namespace hubero
