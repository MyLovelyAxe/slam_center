from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hardli',
    maintainer_email='lijialei829@gmail.com',
    description='Message exchange center for slam',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_hello = slam_center.ros2_zmq_pub:main',
            'send_comp_img = slam_center.comp_img_sub:main',
            'visualize_pcd_cam = slam_center.pcd_cam_vis:main',
        ],
    },
)
