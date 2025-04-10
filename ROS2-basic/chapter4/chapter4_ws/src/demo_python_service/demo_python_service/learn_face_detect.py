import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # 获取功能包share目录绝对路径
import os

def main():
    #package_name = 'demo_python_service'  # 正确的包名
    #package_share_directory = get_package_share_directory(package_name)
    ## 拼接图像文件的路径
    #default_image_path = os.path.join(package_share_directory, 'resource', '1.jpg')
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/1.jpg')
    print(f"图片绝对路径: {default_image_path}")

    image = cv2.imread(default_image_path)
    face_locations = face_recognition.face_locations(
        image, number_of_times_to_upsample=1, model ='hog')
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 4)
    cv2.imshow('Face Detection Result', image)
    cv2.waitKey()
