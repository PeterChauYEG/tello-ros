# tello-ros

build:
`colcon build --packages-select tellobot`
`. install/setup.bash`

run:
`ros2 run tellobot camera_pub_node`
`ros2 run tellobot gui_node`
`ros2 run tellobot pose_ml_node`

launch file:
`ros2 launch src/tellobot/launch/basic_launch.py`
