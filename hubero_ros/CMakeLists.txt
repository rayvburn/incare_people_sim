cmake_minimum_required(VERSION 3.5)
project(hubero_ros)

add_definitions(-std=c++14)

find_package(catkin REQUIRED
	COMPONENTS
		hubero_common
		hubero_interfaces
		hubero_ros_msgs
		cmake_modules
		map_server
		nav_core
		move_base
		move_base_msgs
		nav_msgs
		roscpp
		actionlib
		actionlib_msgs
		tf2
		tf2_ros
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${hubero_common_INCLUDE_DIRS}
	${hubero_interfaces_INCLUDE_DIRS}
	${hubero_ros_msgs_INCLUDE_DIRS}
)

set(HUBERO_NAV_ROS hubero_navigation_ros)
set(HUBERO_TASK_HELPERS hubero_task_helpers)
set(HUBERO_TASK_ROS hubero_task_request_ros)
set(HUBERO_TASK_ROS_API hubero_task_request_ros_api)
set(HUBERO_NODE_ROS hubero_node_ros)
set(HUBERO_ROS_TYPECONV hubero_converter_ros)
set(HUBERO_ROS_MISC hubero_misc_ros)
set(HUBERO_ROS_STATUS hubero_status_ros)

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${HUBERO_NAV_ROS}
		${HUBERO_TASK_HELPERS}
		${HUBERO_TASK_ROS}
		${HUBERO_TASK_ROS_API}
		${HUBERO_NODE_ROS}
		${HUBERO_ROS_STATUS}
	CATKIN_DEPENDS
		hubero_common
		hubero_interfaces
		hubero_ros_msgs
		move_base_msgs
		nav_msgs
		roscpp
		actionlib
		actionlib_msgs
		tf2
		tf2_ros
)

## Node library for HuBeRo
add_library(${HUBERO_NODE_ROS}
	include/${PROJECT_NAME}/node.h
	src/node.cpp
)
target_link_libraries(${HUBERO_NODE_ROS}
	${catkin_LIBRARIES}
)

## Converter library
add_library(${HUBERO_ROS_TYPECONV}
	include/${PROJECT_NAME}/utils/converter.h
	src/utils/converter.cpp
)
target_link_libraries(${HUBERO_ROS_TYPECONV}
	${hubero_common_LIBRARIES}
	${catkin_LIBRARIES}
)

## Generic library with free functions
add_library(${HUBERO_ROS_MISC}
	include/${PROJECT_NAME}/utils/misc.h
	src/utils/misc.cpp
)
target_link_libraries(${HUBERO_ROS_MISC}
	${catkin_LIBRARIES}
)

## Status library
add_library(${HUBERO_ROS_STATUS}
	include/${PROJECT_NAME}/status_ros.h
	src/status_ros.cpp
)
target_link_libraries(${HUBERO_ROS_STATUS}
	${hubero_common_LIBRARIES}
	${hubero_interfaces_LIBRARIES}
	${hubero_ros_msgs_LIBRARIES}
	${HUBERO_NODE_ROS}
	${HUBERO_ROS_TYPECONV}
	${HUBERO_ROS_MISC}
	${catkin_LIBRARIES}
)

## Navigation library for HuBeRo
add_library(${HUBERO_NAV_ROS}
	include/${PROJECT_NAME}/navigation_ros.h
	src/navigation_ros.cpp
)
target_link_libraries(${HUBERO_NAV_ROS}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${HUBERO_ROS_TYPECONV}
	${HUBERO_ROS_MISC}
	${catkin_LIBRARIES}
)

## Task requesting library for HuBeRo
add_library(${HUBERO_TASK_ROS}
	include/${PROJECT_NAME}/task_request_ros.h
	src/task_request_ros.cpp
)
target_link_libraries(${HUBERO_TASK_ROS}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${HUBERO_ROS_TYPECONV}
	${catkin_LIBRARIES}
)

## Task requesting client library for HuBeRo
add_library(${HUBERO_TASK_ROS_API}
	include/${PROJECT_NAME}/task_request_ros_api.h
	src/task_request_ros_api.cpp
)
target_link_libraries(${HUBERO_TASK_ROS_API}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${HUBERO_ROS_TYPECONV}
	${catkin_LIBRARIES}
)

## Functions helping to develop scenarios requiring time/distance synchronization
add_library(${HUBERO_TASK_HELPERS}
	include/${PROJECT_NAME}/task_helpers.h
	src/task_helpers.cpp
)
target_link_libraries(${HUBERO_TASK_HELPERS}
	${HUBERO_TASK_ROS_API}
)

## Install
install(TARGETS ${HUBERO_NAV_ROS} ${HUBERO_TASK_ROS} ${HUBERO_TASK_ROS_API} ${HUBERO_NODE_ROS} ${HUBERO_ROS_STATUS} ${HUBERO_TASK_HELPERS}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

## Test
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(test_task_request test/test_task_request.cpp)
	target_link_libraries(test_task_request ${HUBERO_TASK_ROS})

	catkin_add_gtest(test_navigation test/test_navigation.cpp)
	target_link_libraries(test_navigation ${HUBERO_NAV_ROS})

	catkin_add_gtest(test_misc test/test_misc.cpp)
	target_link_libraries(test_misc ${HUBERO_ROS_MISC})
endif()
