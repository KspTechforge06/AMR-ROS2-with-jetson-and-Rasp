from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_modal_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='Multi-modal perception package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = multi_modal_perception.yolo_detector_node:main',
            'lidar_cluster_node = multi_modal_perception.lidar_cluster_node:main',
            'camera_lidar_association_node = multi_modal_perception.camera_lidar_association_node:main',
            'obstacle_state_classifier_node = multi_modal_perception.obstacle_state_classifier_node:main',
        ],
    },
)