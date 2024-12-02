# hubero_bringup_gazebo_ros

Package contains `.launch` files, `.pgm` maps and `.world` files for evaluation of exemplary scenarios.

## Prerequisites

Before you try to run exemplary worlds, you need to set up the Gazebo simulator with a required set of models (used in provided worlds). Visit the [Gazebo setup wiki page](https://github.com/rayvburn/hubero/wiki/Gazebo-setup) and start setting up the `3DGems` model pack (this one is required, others may be optional depending on your system configuration).

## Run

Before the launch of an exemplary world, one must apply both ROS workspace and Gazebo configurations:

```bash
source <ROS_WS_DIRECTORY>/devel/setup.bash
source /usr/share/<GAZEBO_VER_DIRECTORY>/setup.sh
```

Then, `parking` world, defined for Gazebo, with all necessary Actor interfaces (note that there are 4 actors in this `.world`), can be launched with:

```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=parking rviz:=true
```

If all went OK, after few seconds after the start you should see `move_base` log:

```console
[ INFO] [1643660811.668211218, 8.331000000]: odom received!
```

## Spawning an actor

The crucial question is: how to spawn an `Actor` with a `ActorPlugin` controller in a custom Gazebo world?

### Spawn with a `spawn_model` service

A handy `spawn_actor.launch` was prepared to spawn an actor controlled with the `HuBeRo` plugin in the Gazebo. Once the simulation is running, type:

```sh
roslaunch hubero_bringup_gazebo_ros spawn_actor.launch actor_name:=<ACTOR_UNIQUE_NAME>
```

### Spawn directly in the `.world` file

Typically, Gazebo entities are defined in a `.world` file, so one can also add the `Actor` definition directly to the Gazebo world. Actor definition consists of:

- starting pose definition,
- animation definitions,
- attached sensor relative pose and definition,
- plugin used to control the `Actor`.

Generally, one should use the exemplary worlds definitions as a reference to add actors to their custom worlds. To accomplish that, simply open `living_room.world` or `parking.world` and copy a section that is related to a given actor:

```xml
<actor name="actor1">
  <!-- many lines of actor1 definition:
    - initial pose,
    - sensor
    - animations
    - control plugin
  -->
</actor>
```

**NOTE**: actor names defined in Gazebo `.world` must match names given to the `.launch` file.

## ROS interface

Actors will not have mobility skills without the connection with ROS Navigation stack. Also, Actors will not be able to receive task requests without connection to ROS topics. Use `example.launch` parameter definitions to properly define new Actor interfaces.

## Gazebo world map

To create an ideal map of the Gazebo world one may use `pgm_map_creator`:

- [`hyfan1116/pgm_map_creator`](https://github.com/hyfan1116/pgm_map_creator) with ROS Kinetic and Ubuntu 16.04
- [`rayvburn/pgm_map_creator`](https://github.com/rayvburn/pgm_map_creator.git) with ROS Melodic and Ubuntu 18.04

Examples of use are provided in `maps/*.yaml` files.

One must remember to place/uncomment `collision_map_creator` plugin definition in the `.world` file:

```xml
<plugin name='collision_map_creator' filename='libcollision_map_creator.so'/>
```

## Debugging

`example.launch` provides a possibility to debug the `Actor` controller plugin. First, the user should extend `GAZEBO_PLUGIN_PATH`:

```bash
export GAZEBO_PLUGIN_PATH=<PATH_TO_THE_ROS_WORKSPACE>/devel/.private/hubero_gazebo/lib:$GAZEBO_PLUGIN_PATH
```

Then, run the Gazebo world with the required plugins. ROS1 distro should be adjusted by the user:

```bash
gazebo -s /opt/ros/kinetic/lib/libgazebo_ros_paths_plugin.so -s /opt/ros/kinetic/lib/libgazebo_ros_api_plugin.so <PATH_TO_WORLD_FILE_DIR>/parking.world
```

ROS-part of the launch:

```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=parking rviz:=false gdb:=True
```
