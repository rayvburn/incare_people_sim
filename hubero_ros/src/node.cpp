#include <hubero_ros/node.h>
#include <ros/ros.h>

#include <iostream>

namespace hubero {

bool Node::node_started_ = false;
std::string Node::namespace_;
std::string Node::simulator_frame_id_;
std::shared_ptr<ros::NodeHandle> Node::nh_ptr_;

Node::Node() {
	if (Node::node_started_) {
		return;
	}

	// initialize ROS node
	int argc = 0;
	char **argv = nullptr;
	ros::init(argc, argv, "hubero_ros_node");

	// flag to create only 1 node for all actors
	Node::node_started_ = true;

	// find namespace value
	ros::NodeHandle nh;
	std::string namespace_ros_param;
	nh.searchParam("/hubero_ros/namespace", namespace_ros_param);
	nh.param(namespace_ros_param, Node::namespace_, std::string("hubero_ros"));

	// find simulator_frame value
	std::string sim_frame_ros_param;
	nh.searchParam("/hubero_ros/simulator_frame", sim_frame_ros_param);
	nh.param(sim_frame_ros_param, Node::simulator_frame_id_, std::string("world"));

	// create new NodeHandle in a global namespace
	nh_ptr_.reset(new ros::NodeHandle(("/" + Node::namespace_)));
}

} // namespace hubero
