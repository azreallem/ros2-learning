from setuptools import find_packages, setup
from glob import glob

package_name = 'demo_python_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (target_path, source_path)
        # install/demo_python_service/share/demo_python_service/resource/1.jpg
        ('share/' + package_name + "/resource", ['resource/1.jpg']), 
        ('share/' + package_name + "/resource", ['resource/2.jpg']), 
        ('share/' + package_name + "/launch",  glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gaoliang',
    maintainer_email='azreallem@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detect_service_node = demo_python_service.face_detect_service_node:main',
            'face_detect_client_node = demo_python_service.face_detect_client_node:main',
            'face_detect_node = demo_python_service.face_detect_node:main',
        ],
    },
)
