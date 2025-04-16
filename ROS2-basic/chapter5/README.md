# TF 坐标变换

## 通过命令行使用TF

```bash
ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.0 --z 0.2 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id base_laser

ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_laser --child-frame-id wall_point

ros2 run tf2_ros tf2_echo base_link wall_point
---
At time 0.0
- Translation: [0.400, 0.000, 0.200]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.400
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.200
  0.000  0.000  0.000  1.000

```

```bash
sudo apt-get install ros-rolling-mrpt-apps
3d-rotation-converter
ros2 run tf2_tools view_frames
```


## 对TF原理的简单探究

- 当发布静态广播时，广播的数据就会通过话题通信发布到/tf_static话题上。
- 当发布动态TF时，数据将发布到名称为/tf的话题上。当需要查询坐标变换关系时，则会订阅/tf和/tf_static话题，通过数据中坐标之间的关系计算要查询的坐标之间的关系，这就是TF的工作原理。

```bash
$ ros2 topic list
/parameter_events
/rosout
/tf_static

$ ros2 topic list
/parameter_events
/rosout
/tf_static

$ ros2 topic info /tf_static
Type: tf2_msgs/msg/TFMessage
Publisher count: 2
Subscription count: 0

$ ros2 interface show tf2_msgs/msg/TFMessage
geometry_msgs/TransformStamped[] transforms
        #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        string child_frame_id
        Transform transform
                Vector3 translation
                        float64 x
                        float64 y
                        float64 z
                Quaternion rotation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1

$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 1744436502
      nanosec: 502658766
    frame_id: base_link
  child_frame_id: base_laser
  transform:
    translation:
      x: 0.1
      y: 0.0
      z: 0.2
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1744437023
      nanosec: 85478041
    frame_id: base_laser
  child_frame_id: wall_point
  transform:
    translation:
      x: 0.3
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
```

## 浮点坐标变换

```bash
sudo apt-get install ros-rolling-tf-transformations
pip install transforms3d
```


-------------------------------------------------------------------------


# 可视化工具 rqt和 RViz

```bash
sudo apt-get install ros-rolling-rqt-tf-tree
rm -rf ~/.config/ros.org/rqt_gui.ini
```

# 数据记录工具 ros2 bag

```bash
ros2 bag record /turtle1/cmd_vel
ros2 bag play --help
```






















