from setuptools import setup

package_name = 'socket_test'

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
    description='socket test code',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'socket_pub = socket_test.socket_publisher:main',
            'socket_sub = socket_test.socket_subscriber:main',
            'socket_cli = socket_test.socket_client:main',
            'socket_tcp = socket_test.socket_tcp:main',

        ],
    },
)
