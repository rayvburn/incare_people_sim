cmake_minimum_required(VERSION 3.5)
project(hubero_bringup_gazebo_ros)

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hubero_gazebo
  hubero_ros
)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package()

## Install
install(
  PROGRAMS
    scripts/trajectory_spawner.sh
    scripts/xacro_spawner.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ref: https://github.com/pal-robotics/tiago_robot/blob/kinetic-devel/tiago_bringup/CMakeLists.txt
foreach(dir launch maps rviz worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
