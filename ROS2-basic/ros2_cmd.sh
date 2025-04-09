# basic command

```bash
echo $ROS_DISTRO
source install/setup.zsh
```

# node

```bash
# 列出所有正在运行的节点
ros2 node list
# 查看某个节点提供的服务、话题等
ros2 node info /node_name
```

# Topic

```bash
# 列出当前系统中的所有话题
ros2 topic list
# 查看某个话题的数据类型
ros2 topic type /topic_name
# 查看话题的数据结构定义
ros2 interface show <msg_type>
ros2 interface show std_msgs/msg/String
# 订阅某个话题（打印其消息）
ros2 topic echo /topic_name
# 发布一条消息到某个话题
ros2 topic pub /topic_name std_msgs/msg/String "data: 'hello world'"
# 检查话题连接
ros2 topic info /topic_name
```

# Service

```bash
# 列出所有可用的服务
ros2 service list
ros2 service list | grep service_name
# 查看服务的数据类型
ros2 service type /service_name
# 查看服务的请求与响应格式
ros2 interface show <srv_type>
ros2 interface show std_srvs/srv/Empty
# 调用服务（测试用）
ros2 service call /service_name package_name/srv/ServiceName "{field1: value1, field2: value2}"
```

# Action

```bash
# 列出所有动作服务
ros2 action list
# 查看动作的接口结构
ros2 interface show <action_type>
```

# Interface（msg/srv/action）

```bash
# 查看所有接口
ros2 interface list
# 查看接口定义
ros2 interface show <interface_name>
ros2 interface show std_msgs/msg/Int32
ros2 interface show your_package/srv/YourService
```

# pkg

```bash
# 列出所有已安装的 ROS 2 包
ros2 pkg list
# 查看包的位置
ros2 pkg prefix <package_name>
```
