import rclpy
from rclpy.node import Node
from srv_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
import os

class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.client = self.create_client(FaceDetector, '/face_detect') 
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/2.jpg')
        self.image = cv2.imread(self.default_image_path)
        self.upsample_time = 1
        self.model = 'hog'

    def send_request(self):
        while self.client.wait_for_service(1.0) is False:
            self.get_logger().info('waitting service online ...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'total faces number: {response.number}, use time: {response.use_time}')
        self.show_face_location(response)

    def show_face_location(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top, right, bottom), (255, 0, 0), 2)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey()
 

def main(args = None):
    rclpy.init(args=args)
    node = FaceDetectorClient()
    node.send_request()
    rclpy.shutdown()
