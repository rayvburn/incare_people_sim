#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

using namespace hubero;

const std::string TF_FRAME_REF = "world";

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "parking_scenario_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = 1000 * std::stoi(argv[1]);
	}

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");
	hubero::TaskRequestRosApi actor3("actor3");
	hubero::TaskRequestRosApi actor4("actor4");

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay / 1000);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay / 1000);

	// preparation
	std::function<TaskFeedbackType()> actor4_move_feedback_checker = [&actor4]() -> hubero::TaskFeedbackType {
		return actor4.getMoveToGoalState();
	};
	std::function<TaskFeedbackType()> actor3_move_feedback_checker = [&actor3]() -> hubero::TaskFeedbackType {
		return actor3.getMoveToGoalState();
	};
	std::function<TaskFeedbackType()> actor2_move_feedback_checker = [&actor2]() -> hubero::TaskFeedbackType {
		return actor2.getMoveToGoalState();
	};
	std::function<TaskFeedbackType()> actor1_move_feedback_checker = [&actor1]() -> hubero::TaskFeedbackType {
		return actor1.getMoveToGoalState();
	};

	// =================== 1st stage ========================================
	/* Actor4 and Actor3 go straight to their cars;
	 * Actor2 goes to his friend, Actor1 goes to the dustbin */
	ROS_INFO("[SCENARIO] Firing up the 1st stage!");

	actor4.moveToGoal(Vector3(+0.5, +6.6, 0.0), IGN_PI, TF_FRAME_REF);
	actor3.moveToGoal(Vector3(+0.3, +18.8, 0.0), IGN_PI, TF_FRAME_REF);
	actor2.moveToGoal(Vector3(+7.0, +2.9, 0.0), 0.0, TF_FRAME_REF);
	actor1.moveToGoal(Vector3(+1.2, -5.7, 0.0), IGN_PI, TF_FRAME_REF);

	actor4.startThreadedExecution(std::cref(actor4_move_feedback_checker), "moveToGoal");
	actor3.startThreadedExecution(std::cref(actor3_move_feedback_checker), "moveToGoal");
	actor2.startThreadedExecution(std::cref(actor2_move_feedback_checker), "moveToGoal");
	actor1.startThreadedExecution(std::cref(actor1_move_feedback_checker), "moveToGoal");

	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================

	ROS_INFO("Waiting until Actor2 finishes 'moveToGoal'");
	actor2.join();

	ROS_INFO("[SCENARIO] Firing up the 2nd stage!");

	/* Actor2 talks for few seconds */
	ROS_INFO("Actor2 got to the place! He will start talking to the person nearby!");
	actor2.talk(Vector3(+7.0, +2.9, 0.0), IGN_DTOR(90), TF_FRAME_REF);
	hubero::TaskRequestRosApi::wait(std::chrono::milliseconds(5000));

	ROS_INFO("Actor2 finishes talking to the person nearby!");
	actor2.stopTalking();

	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================

	// actor1 will most likely go for a longer time than actor2
	ROS_INFO("Waiting until Actor1 finishes 'moveToGoal'");
	actor1.join();
	ROS_INFO("Actor1 threw the rubbish away to the dumpster!");

	ROS_INFO("[SCENARIO] Firing up the 3rd stage!");

	/* Actor1 and Actor2 go to their cars */
	ROS_INFO("Actor1 and Actor2 start moving towards their cars!");
	actor2.moveToGoal(Vector3(-3.5, +10.5, 0.0), IGN_PI, TF_FRAME_REF);
	actor1.moveToGoal(Vector3(-0.5, +14.8, 0.0), IGN_PI, TF_FRAME_REF);
	actor2.startThreadedExecution(actor2_move_feedback_checker, "moveToGoal");
	actor1.startThreadedExecution(actor1_move_feedback_checker, "moveToGoal");

	// the requests are processed is separate threads
	ROS_INFO("Actor1 and Actor2 are moving towards cars! Waiting until they reach their goal");
	actor1.join();
	actor2.join();

	actor3.join();
	actor4.join();

	ROS_INFO("[SCENARIO] 3rd stage completed!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	return 0;
}
