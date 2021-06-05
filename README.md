# tello-ros

pre:
import h264decoder
install from github
https://github.com/DaWelter/h264decoder
python3 -m pip install ./downloads/SomeProject-1.0.4.tar.gz
python3 -m pip install <path>

$ export OPENSSL_ROOT_DIR=/usr/local/opt/openssl
$ export NDDSHOME=/Applications/rti_connext_dds-6.1.0

dev:
`pylint --rcfile=~/robotics/ros-drone/.pylintrc *.py`

build:
`colcon build --packages-select tellobot`
`colcon build --packages-select tellobot`
`. install/setup.bash`

run:
`ros2 run tellobot camera_pub_node`
`ros2 run tellobot gui_node`
`ros2 run tellobot pose_ml_node`

launch file:
`ros2 launch tellobot basic.launch.py`
`ros2 launch tellobot tello_camera.launch.py`

ml:
# TensorFlow implementation of SSD model from https://github.com/tensorflow/models/tree/master/research/object_detection
ssd_tf:
load_info:
url: "http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2017_11_17.tar.gz"
sha1: "9e4bcdd98f4c6572747679e4ce570de4f03a70e2"
download_sha: "6157ddb6da55db2da89dd561eceb7f944928e317"
download_name: "ssd_mobilenet_v1_coco_2017_11_17.tar.gz"
member: "ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb"
model: "ssd_mobilenet_v1_coco_2017_11_17.pb"
config: "ssd_mobilenet_v1_coco_2017_11_17.pbtxt"
mean: [0, 0, 0]
scale: 1.0
width: 300
height: 300
rgb: true
classes: "object_detection_classes_coco.txt"
sample: "object_detection"

