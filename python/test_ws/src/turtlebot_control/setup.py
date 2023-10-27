from setuptools import setup

package_name = 'turtlebot_control'

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
    maintainer='bat20',
    maintainer_email='sigma6317@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_joystick = turtlebot_control.turtlebot_joystick:main',
            'joystick_control = turtlebot_control.joystick_control:main',
            'test = turtlebot_control.test:main',
            'test2 = turtlebot_control.test2:main',
            'test3 = turtlebot_control.test3:main',
        ],
    },
)
