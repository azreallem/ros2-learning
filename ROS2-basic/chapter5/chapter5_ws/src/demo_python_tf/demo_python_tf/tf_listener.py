import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler # 欧拉角转四元数函数
from tf_transformations import euler_from_quaternion

class TFListener(Node):

    def __init__(self):
     super().__init__('tf_listener')
     self.tf_buffer = Buffer()
     self.tf_listener = TransformListener(self.tf_buffer, self)
     self.timer_ = self.create_timer(1.0, self.timer_callback)
     self.get_logger().info('TFListener node has been started')

     def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'bottle_link', rclpy.time.Time())
            trans_euler = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            self.get_logger().info('Euler angles: {}'.format(trans_euler))
            self.get_logger().info('Translation: {}'.format(trans.transform.translation))
            self.get_logger().info('Rotation: {}'.format(trans.transform.rotation))
        except Exception as e:
            self.get_logger().error('Error: {}'.format(e))

def main():
    rclpy.init()
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    rclpy.shutown()
