from setuptools import setup

package_name = 'opencv_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='sigma6317@gmail.com',
    description='ros2 foxy python opencv',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_pub = opencv_python.image_pub:main",
            "image_sub = opencv_python.image_sub:main",
        ],
    },
)
