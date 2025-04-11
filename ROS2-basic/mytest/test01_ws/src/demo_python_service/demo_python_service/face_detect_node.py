import rclpy
from rclpy.node import Node
from srv_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
import os
from rcl_interfaces.msg import SetParametersResult

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.bridge = CvBridge()
        self.service = self.create_service(FaceDetector, '/face_detect', self.face_detect_callback) 
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/1.jpg')
        self.declare_parameter('upsample_time', 1)
        self.declare_parameter('model', 'hog')
        self.upsample_time =  self.get_parameter('upsample_time').value
        self.model =  self.get_parameter('model').value
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            self.get_logger().info(f"Set callback: {param.name} = {param.value}")
            if param.name == 'upsample_time':
                self.upsample_time = param.value
            if param.name == 'model':
                self.model = param.value
        return SetParametersResult(successful=True)

    def face_detect_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image) # ros_img <==cvbridge==> cv_img
        else:
            cv_image = cv2.imread(self.default_image_path)
        start_time = time.time()
        self.get_logger().info('Finish load image, start face detect ...')
        face_locations = face_recognition.face_locations(cv_image, self.upsample_time, self.model)
        speed_time = time.time() - start_time
        self.get_logger().info(f'Finish face detect, speed time: {speed_time} ...')
        # config response struct: FaceDetector
        response.number = len(face_locations)
        response.use_time = speed_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response

def main(args = None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()
