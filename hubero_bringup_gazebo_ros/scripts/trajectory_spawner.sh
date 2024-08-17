#!/usr/bin/env bash
#
# This is a script for spawning a non-HuBeRo actor (generic Gazebo actor) with a predefined trajectory to be executed
#
# NOTE: the contents of the input_file (2nd argument) should be in the following form:
#
# <?xml version="1.0"?>
# <sdf version="1.6">
# <!-- NOTE: SCRIPTDELETE is placed here as it must be deleted from the resultant .xml to be properly loaded by the Gazebo -->
# <SCRIPTDELETE xmlns:xacro="http://www.ros.org/wiki/xacro">
#    <!-- Ref: http://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot -->
#    <!-- Also, namespace for ROS topics related to the actor -->
#    <xacro:arg name="actor_name" default="actor" />
#    <!-- Arg->Property hack -->
#    <xacro:property name="actor_name" value="$(arg actor_name)" />
#
#     <actor name="${actor_name}">
#       <skin>
#         <filename>walk.dae</filename>
#       </skin>
#       <animation name="walking">
#         <filename>walk.dae</filename>
#         <interpolate_x>true</interpolate_x>
#       </animation>
#       <script>
#         <trajectory id="0" type="walking">
#           <waypoint>
#             <time>0</time>
#             <pose>0 2 0 0 0 -1.57</pose>
#           </waypoint>
#           <waypoint>
#             <!--
#             ...
#             -->
#           </waypoint>
#         </trajectory>
#       </script>
#     </actor>
# </SCRIPTDELETE>
# </sdf>
#

# Pass arguments
actor_name="$1"
# xacro/URDF file location
input_file="$2"

# compose arguments to be passed to the xacro
XACRO_ARGS="actor_name:=${actor_name}"

# deletes lines containing SCRIPTDELETE phrase
SDF=$(xacro "$input_file" $XACRO_ARGS | sed "/SCRIPTDELETE/d")

# save sdf to file
SDF_FILE="/tmp/trajectory_spawner_${actor_name}"
echo $SDF > $SDF_FILE

# spawn coords are defined in the xacro
rosrun gazebo_ros spawn_model -sdf -file $SDF_FILE -model $actor_name
