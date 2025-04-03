from setuptools import find_packages, setup
import os, glob

package_name = 'yolov5_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name +'/urdf', glob.glob(os.path.join('urdf', '*.urdf.xacro'))),
        ('share/' + package_name +'/config', glob.glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name +'/config', glob.glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='s',
    maintainer_email='s@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'yolo_detector = {package_name}.yolov5_detector:main',
            f'find_contours = {package_name}.find_contours:main',
            f'realsense_test = {package_name}.realsense_test:main',
            f'find_contours_canny = {package_name}.find_contours_canny:main',
        ],
    },
)
