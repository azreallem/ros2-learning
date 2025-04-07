# chapter04

## commands

```bash
ros2 service list -t
ros2 interface show turtlesim_msgs/srv/Spawn
ros2 service call /spawn turtlesim_msgs/srv/Spawn "{x: 1, y: 1}"
rqt
ros2 service list -t | grep parameter
ros2 param list
ros2 param describe /turtlesim background_r
ros2 param get /turtlesim background_r
ros2 param set /turtlesim background_r 255
ros2 param dump /turtlesim > turtlesim_param.yaml
```




