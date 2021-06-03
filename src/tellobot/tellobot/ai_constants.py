from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE

MAX_HEAD_Y=GUI_CENTER_BOX_HALF_SIZE + (GUI_CENTER_BOX_HALF_SIZE/4)
MIN_HEAD_Y=GUI_CENTER_BOX_HALF_SIZE
POSE_CENTERED_SENSITIVITY=10

POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11],
              [11, 12], [12, 13]]
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

    # =======
    'left_arm_v': 'left_arm_v',  # 120 < shoulder_angle < 180
    'right_arm_v': 'right_arm_v',  # 20 < shoulder_angle < 80
}


POSE_PROTOFILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/pose/mpi/pose_deploy_linevec_faster_4_stages.prototxt"
POSE_WEIGHTSFILE = "/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/models/pose/mpi/pose_iter_160000.caffemodel"
