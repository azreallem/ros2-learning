import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
import os

class FaceDetectorionNode(Node) :
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge()
        self.service = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.get_logger().info(f"self.service:{self.service}")

        package_name = 'demo_python_service'  # 正确的包名
        package_share_directory = get_package_share_directory(package_name)
        # 拼接图像文件的路径
        self.default_image_path = os.path.join(package_share_directory, 'resource', '1.jpg')

        #self.default_image_path = get_package_share_directory('demo_python_service') + '/resource/1.jpg'
        self.upsample_times = 1
        self.model = "hog"
        self.get_logger().info("waiting detect.")

    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
        start_time = time.time()
        self.get_logger().info("Fininshed load image, start detect.")
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        end_time = time.time()
        self.get_logger().info(f"Fininshed detect, speed time:{end_time-start_time}")
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()
