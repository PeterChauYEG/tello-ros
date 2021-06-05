from launch import LaunchDescription, substitutions, actions
from launch_ros.actions import Node

logger = substitutions.LaunchConfiguration("log_level")

def generate_launch_description():
    return LaunchDescription([
        actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        Node(
            package='tellobot',
            node_executable='gui_camera_node',
            name='gui_camera_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='tellobot',
            node_executable='pose_ml_node',
            name='pose_ml_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='tellobot',
            node_executable='ai_node',
            name='ai_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='tellobot',
            node_executable='gui_buttons_node',
            name='gui_buttons_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
        Node(
            package='tellobot',
            node_executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger]
        ),
    ])
