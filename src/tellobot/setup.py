from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tellobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'libh264decoder'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peterchau',
    maintainer_email='peter@labone.tech',
    description='Tello robot',
    license='MIT',
    tests_require=['pytest'],
    package_data={'libh264decoder': ['libh264decoder/libh264decoder.so']},
    include_package_data=True,

    entry_points={
        'console_scripts': [
            'gui_buttons_node = tellobot.gui_buttons_node:main',
            'gui_camera_node = tellobot.gui_camera_node:main',
            'camera_node = tellobot.camera_node:main',
            'drone_node = tellobot.drone_node:main',
            'pose_ml_node = tellobot.pose_ml_node:main',
            'ai_node = tellobot.ai_node:main'
        ],
    },
)
