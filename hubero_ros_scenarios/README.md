# hubero_ros_scenarios

Exemplary scenarios used for HuBeRo evaluation.

Scenarios are related to exemplary worlds provided in the `hubero_bringup_gazebo_ros` package - `parking` and `living_room`.

## Run

To evaluate provided scenarios, run commands provided below.

An extra argument to scenario node (e.g. `3`) is a delay (in seconds) of scenario start. Delay starts once all ROS `Action Clients` connect to their ROS `Action Servers` (which may take N-teen seconds for 4 actors).

### `parking` scenario

Run Gazebo with `Actor` controllers (`actor1`, `actor2`, `actor3`, `actor4`), their ROS interfaces:

```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=parking rviz:=false
```

In the second terminal, run scenario execution:

```bash
rosrun hubero_ros_scenarios parking_node 3
```

The `parking` scenario is implemented relying on tasks execution in separate threads.

### `living room` scenario


```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=living_room rviz:=false
```

In the second terminal, run scenario execution:

```bash
rosrun hubero_ros_scenarios living_room_node 3
```

The `living room` scenario is implemented relying on manual evaluating of the task states (threaded executor is not used, in contrary to the `parking` scenario).
