from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE

MAX_HEAD_Y = GUI_CENTER_BOX_HALF_SIZE + GUI_CENTER_BOX_HALF_SIZE
MIN_HEAD_Y = GUI_CENTER_BOX_HALF_SIZE
POSE_CENTERED_SENSITIVITY = 20

OBJECT_DETECTION_SENSITIVITY = 50
POSE_DETECTION_SENSITIVITY = 0.05

POSE_PAIRS = [
  [0, 1],
  [1, 2],
  [2, 3],
  [3, 4],
  [1, 5],
  [5, 6],
  [6, 7],
  [1, 14],
  [14, 8],
  [8, 9],
  [9, 10],
  [14, 11],
  [11, 12],
  [12, 13]
]
OVERFLOW_NULL = -999
POSES = {
  'arms_down_45': 'arms_down_45',
  'arms_flat': 'arms_flat',
  'arms_V': 'arms_V',
  'arms_up_45': 'arms_up_45',

  'left_arm_down_45': 'left_arm_down_45',  # 225 202.5,247.5
  'left_arm_flat': 'left_arm_flat',  # 180 202.5,157.5
  'left_arm_up_45': 'left_arm_up_45',  # 135 157.5,112.5

  'right_arm_down_45': 'right_arm_down_45',  # -45 -67.5,-22.5
  'right_arm_flat': 'right_arm_flat',  # 0 -22.5,22.5
  'right_arm_up_45': 'right_arm_up_45',  # 45 67.5,22.5

  'left_arm_v': 'left_arm_v',  # 120 < shoulder_angle < 180
  'right_arm_v': 'right_arm_v',  # 20 < shoulder_angle < 80
}

# pylint: disable=line-too-long
POSE_PROTO_FILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/pose/mpi/pose_deploy_linevec_faster_4_stages.prototxt"
# pylint: disable=line-too-long
POSE_WEIGHTS_FILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/pose/mpi/pose_iter_160000.caffemodel"

OBJECT_DETECTION_MODEL_FILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb"
OBJECT_DETECTION_CONFIG_FILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/ssd_mobilenet_v1_coco_2017_11_17/ssd_mobilenet_v1_coco_2017_11_17.pbtxt"
OBJECT_DETECTION_CLASSES_FILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/ssd_mobilenet_v1_coco_2017_11_17/object_detection_classes_coco.txt"

IMAGES_DIR_PATH = '/Users/peterchau/robotics/ros-drone/src/images'
