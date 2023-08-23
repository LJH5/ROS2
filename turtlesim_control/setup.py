from setuptools import setup

package_name = 'turtlesim_control'

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
    maintainer_email='bat20@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_sub = turtlesim_control.turtlesim_sub:main',
            'turtlesim_pub = turtlesim_control.turtlesim_pub:main',
            'turtlesim_keyboard = turtlesim_control.turtlesim_keyboard:main',
            'turtlesim_joystick = turtlesim_control.turtlesim_joystick:main',
        ],
    },
)
