from setuptools import setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Robot control node for MyCobot 280',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = robot_control.robot_control_node:main',
            'test_pose_publisher = robot_control.test_pose_publisher:main',

        ],
    },
)

