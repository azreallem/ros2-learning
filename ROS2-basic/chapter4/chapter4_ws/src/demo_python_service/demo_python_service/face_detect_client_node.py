import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import os
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class FaceDetectorionClient(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test1_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/2.jpg')
        self.image = cv2.imread(self.test1_image_path)

    def call_set_parameters(self, parameters):
        # 1.创建一个客户端，等待服务上线
        update_param = self.create_client(SetParameters, '/face_detection_node/set_parameters')
        while update_param.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待参数更新服务端上线')
        # 2.创建request
        request = SetParameters.Request()
        request.parameters = parameters
        # 3.调用服务端更新参数
        future = update_param.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response

    def update_detect_model(self, model='hog'):
        # 1.创建参数对象
        param = Parameter()
        param.name = 'model'
        # 2. 赋值
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        param.value = param_value
        # 3. 请求更新参数
        response = self.call_set_parameters([param])
        # 4. 打印更新结果
        for result in response.results:
            self.get_logger().info(f"设置参数结果: {result.successful} {result.reason}")

    def send_request(self):
        # 1. 判断服务端是否在线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'waitting service ...')
        # 2. 构造Request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 3. 发送请求并等待处理完成
        future = self.client.call_async(request) # 现在future并没有包含响应结果，需要等待服务端处理完成才会把结果放到future中
        rclpy.spin_until_future_complete(self, future) # 等待服务端返回响应
        response = future.result()
        self.get_logger().info(f'accept response: the image has total: {response.number} faces, speed time: {response.use_time}')
#        self.show_face_locations(response)

    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 4)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectorionClient()
    face_detect_client.update_detect_model('hog')
    face_detect_client.send_request()
    face_detect_client.update_detect_model('cnn')
    face_detect_client.send_request()
    rclpy.shutdown()
