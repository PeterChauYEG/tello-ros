from setuptools import setup

package_name = 'tellobot'

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
    maintainer='peterchau',
    maintainer_email='peter@labone.tech',
    description='Tello robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub_node = tellobot.camera_pub_node:main',
            'gui_node = tellobot.gui_node:main',
            'pose_ml_node = tellobot.pose_ml_node:main',
            'ai_node = tellobot.ai_node:main',
            'gui_buttons_node = tellobot.gui_buttons_node:main',
            'drone_node = tellobot.drone_node:main'
        ],
    },
)
