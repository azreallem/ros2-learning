import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
import os
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

class FaceDetectorionNode(Node) :
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge() # ROS <====== CV BRIDGE =====> OPENCV
        self.service = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.get_logger().info(f"self.service:{self.service}")
        # 参数申明
        self.declare_parameter('number_of_times_to_upsample', 1)
        self.declare_parameter('model', 'hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value
 
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/1.jpg')
        self.upsample_times = 1
        self.model = "hog"
        self.get_logger().info("waiting detect.")
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f"{parameter.name}->{parameter.value}")
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            if parameter.name == 'model':
                self.model = parameter.value
        return SetParametersResult(successful=True)

    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
        # cv_image 已经是一个opencv格式的图像了
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
        return response # 必须返回response

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()
