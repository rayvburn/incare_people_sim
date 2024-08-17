#!/usr/bin/env bash
#
# Pass xacro arguments one by one, beware of the order
actor_name="$1"
pose_x="$2"
pose_y="$3"
pose_z="$4"
pose_roll="$5"
pose_pitch="$6"
pose_yaw="$7"
visualize_laser="$8"
skin_file="$9"

# compose arguments to be passed to the xacro
XACRO_ARGS="actor_name:=${actor_name} pose_x:=${pose_x} pose_y:=${pose_y} pose_z:=${pose_z} pose_roll:=${pose_roll} pose_pitch:=${pose_pitch} pose_yaw:=${pose_yaw} visualize_laser:=${visualize_laser} skin_file:=${skin_file}"

# deletes lines containing SCRIPTDELETE phrase
SDF=$(xacro "$(rospack find hubero_gazebo)/urdf/actor.xacro" $XACRO_ARGS | sed "/SCRIPTDELETE/d")

# save sdf to file
SDF_FILE="/tmp/xacro_spawner_${actor_name}"
echo $SDF > $SDF_FILE

# spawn coords are defined in the xacro
rosrun gazebo_ros spawn_model -sdf -file $SDF_FILE -model $actor_name
