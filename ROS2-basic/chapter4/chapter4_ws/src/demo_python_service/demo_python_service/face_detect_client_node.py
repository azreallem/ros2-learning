import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import os

class FaceDetectorionClient(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test1_image_path = get_package_share_directory('demo_python_service') + '/resource/2.jpg'
        self.image = cv2.imread(self.test1_image_path)

    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'waitting service ...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'accept response: the image has total: {response.number} faces, speed time: {response.use_time}')
        self.show_face_locations(response)

        #def request_callback(result_future):
        #    response = result_future.result()
        #    self.get_logger().info(f'accept response: the image has total: {response.number} faces, speed time: {response.use_time}')
        #    self.show_face_locations(response)
        #future.add_done_callback(request_callback)
        # then add `rclpy.spin(face_detect_client)` to main

    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectorionClient()
    face_detect_client.send_request()
    rclpy.shutdown()
