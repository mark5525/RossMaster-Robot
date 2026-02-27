from setuptools import setup

package_name = 'rossmaster_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rossmaster_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RossMaster Robot ROS2 wrapper',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver_node = rossmaster_robot.driver_node:main',
            'odom_node = rossmaster_robot.odom_node:main',
        ],
    },
)
