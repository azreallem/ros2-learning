# notes

ROS2 四大核心通信机制:话题（Topic）、服务（Service）、参数（Parameter）、动作（Action）

- 话题通信：是publisher和subscription之间的通信，是单向传输的，subscription有一个callback回调函数，可以添加一个timer定时器，或者可以用于循环处理从订阅的话题中获取到的message。

- 服务通信：是基于请求和响应的双向通信机制。service有callback回调函数处理，client没有callback。

- 参数通信：是用于管理节点的设置，参数的通信机制是基于服务通信实现的。`ros2 service list -t | grep parameter`


# terminal

```bash
# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [9:53:23]
$ ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim_msgs/msg/Color]
/turtle1/pose [turtlesim_msgs/msg/Pose]

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [9:53:36]
$ ros2 service list -t
/clear [std_srvs/srv/Empty]
/kill [turtlesim_msgs/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim_msgs/srv/Spawn]
/turtle1/set_pen [turtlesim_msgs/srv/SetPen]
/turtle1/teleport_absolute [turtlesim_msgs/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim_msgs/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [9:53:43]
$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [9:54:07]
$ ros2 interface show turtlesim_msgs/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [10:00:11] C:2
$ ros2 param list
/turtlesim:
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [10:00:18]
$ ros2 service --help
Commands:
  call  Call a service
  echo  Echo a service
  find  Output a list of available services of a given type
  info  Print information about a service
  list  Output a list of available services
  type  Output a services type

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [10:01:39]
$ ros2 topic --help
Commands:
  bw     Display bandwidth used by topic
  delay  Display delay of topic from timestamp in header
  echo   Output messages from a topic
  find   Output a list of available topics of a given type
  hz     Print the average publishing rate to screen
  info   Print information about a topic
  list   Output a list of available topics
  pub    Publish a message to a topic
  type   Print a topics type

# gaoliang @ gaoliang-WUJIE14XA in ~/work/ros2-learning on git:master x [10:01:53]
$ ros2 param --help
Commands:
  delete    Delete parameter
  describe  Show descriptive information about declared parameters
  dump      Show all of the parameters of a node in a YAML file format
  get       Get parameter
  list      Output a list of available parameters
  load      Load parameter file for a node
  set       Set parameter

```
