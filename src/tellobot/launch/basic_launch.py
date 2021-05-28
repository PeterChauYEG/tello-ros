from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tellobot',
            node_executable='camera_pub_node',
            name='camera_pub_node'
        ),
        Node(
            package='tellobot',
            node_executable='gui_node',
            name='gui_node'
        ),
        Node(
            package='tellobot',
            node_executable='pose_ml_node',
            name='pose_ml_node'
        ),
    ])
