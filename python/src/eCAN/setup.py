from setuptools import find_packages, setup

package_name = 'eCAN'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr2',
    maintainer_email='amr2@todo.todo',
    description='TODO: Package description',
    license='Apatch2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ecan_client=eCAN.ecan_client:main",
            "server_test=eCAN.server_test:main",
        ],
    },
)
