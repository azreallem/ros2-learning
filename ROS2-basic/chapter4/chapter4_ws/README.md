# env config

Python 3.12.3

```bash
# 导出已安装的包
##pip freeze > requirements.txt
python3 -m venv ~/python3_venv
source ~/python3_venv/bin/activate
pip install -r requirements.txt
./build.sh
```

# run

```bash
source ~/python3_venv/bin/activate
source install/setup.zsh
ros2 run demo_python_service face_detect_node
ros2 interface list
ros2 interface show chapter4_interfaces/srv/FaceDetector
ros2 service list
ros2 service call /face_detect chapter4_interfaces/srv/FaceDetector
ros2 run demo_python_service face_detect_client_node
ros2 param list
```
