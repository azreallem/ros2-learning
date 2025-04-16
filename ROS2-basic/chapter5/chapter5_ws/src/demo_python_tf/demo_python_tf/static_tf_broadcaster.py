import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster       # 静态坐标发布器
from geometry_msgs.msg import TransformStamped       # 消息接口
from tf_transformations import quaternion_from_euler # 欧拉角转四元数函数

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera_link"

        # 设置平移部分
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.5
        transform.transform.translation.z = 0.5
 
        # 欧拉角转四元数 q=x,y,z,w
        q = quaternion_from_euler(math.radians(180), 0, 0)
 
        # 旋转部分进行赋值
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
 
        # 静态坐标关系发布
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info(f'发布 TF: {transform}')

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()
