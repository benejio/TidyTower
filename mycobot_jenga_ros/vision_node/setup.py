from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Vision Node for Jenga Block Detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.vision_node:main'
        ],
    },
)

