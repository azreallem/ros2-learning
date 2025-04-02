# Tutorials

## Understanding nodes

A node is a fundamental ROS 2 element. Each node in ROS should be responsible for a single, modular purpose. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

### Tasks

#### ros2 run

```bash
#ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

####  ros2 node list

```bash
ros2 node list
```

##### Remapping
Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

#### ros2 node info

`ros2 node info` returns a list of subscribers, publishers, services, and actions.

```bash
#ros2 node info <node_name>
ros2 node info /turtlesim
```

------------------------------------------

## Understanding topics

ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

### Task

#### rqt graph
You can also open rqt_graph by opening `rqt` and selecting `Plugins > Introspection > Node Graph`.

#### `ros2 topic list`

```bash
ros2 topic list
ros2 topic list -t
ros2 topic echo /turtle1/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

#### `ros2 topic pub`

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --once -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
# --once: “publish one message then exit”.
# -w 2: “wait for two matching subscriptions”.
```

#### `ros2 topic hz` and `ros2 topic bw`

```bash
ros2 topic hz /turtle1/pose
ros2 topic bw /turtle1/pose
```

#### `ros2 topic find`

```bash
#ros2 topic find <topic_type>
ros2 topic find geometry_msgs/msg/Twist
```

-----------------------

## UnderStanding services



