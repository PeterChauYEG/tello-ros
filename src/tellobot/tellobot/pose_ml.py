import math
import time
from threading import Thread

import cv2
from tellobot.ai_constants import OVERFLOW_NULL, POSES, POSE_PROTOFILE, POSE_WEIGHTSFILE

# pylint: disable=too-many-public-methods
class PoseML:
  def __init__(self):

    # statics ============
    self.prob_threshold = 0.05

    # input image dimensions for the network
    # IMPORTANT:
    # Greater inWidth and inHeight will result in higher accuracy but longer process time
    # Smaller inWidth and inHeight will result in lower accuracy but shorter process time
    # Play around it by yourself to get best result!
    # https://learnopencv.com/deep-learning-based-human-pose-estimation-using-opencv-cpp-python/

    # og
    # inWidth = 168
    # inHeight = 168
    self.input_w = 128
    self.input_h = 128

    # total number of the skeleton nodes
    self.nPoints = 15

    # init vars ===========
    # input frame
    self.frame_w = None
    self.frame_h = None

    # count the number of frames
    self.frame_cnt = 0

    # posses detected
    self.current_poses_captured = {}

    # the period of pose reconigtion,it depends on your computer performance
    self.period = 0

    # record how many times the period of pose reconigtion called
    self.period_calculate_cnt = 0
    self.frame_cnt_threshold = 0
    self.current_pose_captured_threshold = 0

    # detection return
    self.current_pose = ''
    self.points = []
    self.current_frame = None
    self.thread_started = False

    # init ===============
    # read the neural network of the pose recognition
    self.net = cv2.dnn.readNetFromCaffe(POSE_PROTOFILE, POSE_WEIGHTSFILE)

    self.thread = Thread(target=self.update, args=(), daemon=True)
    self.start()

  def set_current_frame(self, frame):
    self.current_frame = frame

  def start(self):
    self.thread_started = True
    self.thread.start()

  def update(self):
    while True:
      if not self.thread_started:
        return

      if self.current_frame is not None:
        self.detect()
        self.current_frame = None

  def read(self):
    return self.current_pose, self.points

  def stop(self):
    self.thread_started = False

  @staticmethod
  def get_angle(start, end):
    """
    Calculate the angle between start and end

    :param start: start point [x, y]
    :param end: end point [x, y]
    :return: the clockwise angle from start to end
    """
    angle = int(math.atan2((start[1] - end[1]), (start[0] - end[0])) * 180 / math.pi)
    return angle

  def get_left_arm_position(self, min_angle, max_angle, is_straight, is_v=False):
    left = False

    if OVERFLOW_NULL not in (self.points[5][0], self.points[6][0], self.points[7][0]):
      shoulder_angle = self.get_angle(self.points[5], self.points[6])
      # correct the dimension
      if shoulder_angle < 0:
        shoulder_angle = shoulder_angle + 360

      if min_angle < shoulder_angle < max_angle:
        elbow_angle = self.get_angle(self.points[6], self.points[7])
        if elbow_angle < 0:
          elbow_angle = elbow_angle + 360

        if is_straight:
          if abs(elbow_angle - shoulder_angle) < 25:
            left = True

        if is_v:
          if 90 < elbow_angle < 180:
            left = True

    return left

  def get_right_arm_position(self, min_angle, max_angle, is_straight, is_v=False):
    right = False

    if OVERFLOW_NULL not in (self.points[2][0], self.points[3][0], self.points[4][0]):
      shoulder_angle = self.get_angle(self.points[2], self.points[3])
      if min_angle < shoulder_angle < max_angle:
        elbow_angle = self.get_angle(self.points[2], self.points[3])

        if is_straight:
          if abs(elbow_angle - shoulder_angle) < 25:
            right = True

        if is_v:
          if 0 < elbow_angle < 90:
            right = True

    return right

  def is_left_arm_up_45(self):
    return self.get_left_arm_position(113, 157, True)

  def is_right_arm_up_45(self):
    return self.get_right_arm_position(23, 67, True)

  def is_left_arm_down_45(self):
    return self.get_left_arm_position(203, 247, True)

  def is_right_arm_down_45(self):
    return self.get_right_arm_position(-67, -23, True)

  def is_left_arm_flat(self):
    return self.get_left_arm_position(158, 202, True)

  def is_right_arm_flat(self):
    return self.get_right_arm_position(-22, 22, True)

  def is_arms_up_45(self):
    is_right_arm_in_position = self.get_right_arm_position(23, 67, True)
    is_left_arm_in_position = self.get_right_arm_position(113, 157, True)

    if is_right_arm_in_position and is_left_arm_in_position:
      return True

    return False

  def is_arms_down_45(self):
    is_right_arm_in_position = self.get_right_arm_position(-67, -23, True)
    is_left_arm_in_position = self.get_right_arm_position(203, 247, True)

    if is_right_arm_in_position and is_left_arm_in_position:
      return True

    return False

  def is_arms_flat(self):
    is_right_arm_in_position = self.get_right_arm_position(-22, 22, True)
    is_left_arm_in_position = self.get_right_arm_position(158, 202, True)

    if is_right_arm_in_position and is_left_arm_in_position:
      return True

    return False

  def is_arms_v(self):
    is_right_arm_in_position = self.get_right_arm_position(-67, 23, False, True)
    is_left_arm_in_position = self.get_right_arm_position(203, 247, False, True)

    if is_right_arm_in_position and is_left_arm_in_position:
      return True

    return False

  def preprocess(self, frame):
    frame = cv2.bilateralFilter(frame, 5, 50, 100)

    if self.frame_w is None or self.frame_h is None:
      self.frame_w = frame.shape[1]
      self.frame_h = frame.shape[0]

    frame_blob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (self.input_w, self.input_h),
                                       (0, 0, 0), swapRB=False, crop=False)
    self.net.setInput(frame_blob)

    return frame

  def calculate_period(self, period_start_time, period_end_time):
    if self.period_calculate_cnt <= 5:
      self.period = self.period + period_end_time - period_start_time
      self.period_calculate_cnt = self.period_calculate_cnt + 1
    if self.period_calculate_cnt >= 6:
      self.period = self.period / 6

  def set_detection_thresholds(self):
    if self.period < 0.3:
      self.frame_cnt_threshold = 5
      self.current_pose_captured_threshold = 4
    elif 0.3 <= self.period < 0.6:
      self.frame_cnt_threshold = 4
      self.current_pose_captured_threshold = 3
    elif self.period >= 0.6:
      self.frame_cnt_threshold = 2
      self.current_pose_captured_threshold = 2

  def clear_detection_period_state(self):
    self.frame_cnt = 0
    self.current_poses_captured = {}

  def clear_detection_state(self):
    self.current_pose = ''
    self.points = []

  def calculate_period_pose(self):
    if self.frame_cnt >= self.frame_cnt_threshold:
      if len(self.current_poses_captured) != 0:
        # pylint: disable=line-too-long
        self.current_pose = max(self.current_poses_captured, key=lambda k: self.current_poses_captured[k])

        # we need a map of pose to pose
        if self.current_pose != '':
          print(self.current_pose)

      self.clear_detection_period_state()

  def update_poses_captured(self, key):
    if key not in self.current_poses_captured:
      self.current_poses_captured[key] = 0
    self.current_poses_captured[key] += 1

  def calculate_pose(self):
    if self.is_arms_down_45():
      self.update_poses_captured(POSES['arms_down_45'])
    elif self.is_arms_flat():
      self.update_poses_captured(POSES['arms_flat'])
    elif self.is_arms_v():
      self.update_poses_captured(POSES['arms_V'])
    elif self.is_arms_up_45():
      self.update_poses_captured(POSES['arms_up_45'])
    elif self.is_left_arm_up_45():
      self.update_poses_captured(POSES['left_arm_up_45'])
    elif self.is_right_arm_up_45():
      self.update_poses_captured(POSES['right_arm_up_45'])
    elif self.is_left_arm_down_45():
      self.update_poses_captured(POSES['left_arm_down_45'])
    elif self.is_right_arm_down_45():
      self.update_poses_captured(POSES['right_arm_down_45'])
    elif self.is_left_arm_flat():
      self.update_poses_captured(POSES['left_arm_flat'])
    elif self.is_right_arm_flat():
      self.update_poses_captured(POSES['right_arm_flat'])

  def handle_pose_points(self, output):
    # get shape of the output
    H = output.shape[2]
    W = output.shape[3]

    for i in range(self.nPoints):
      # confidence map of corresponding body's part.
      probMap = output[0, i, :, :]

      # Find global maxima of the probMap.
      _, prob, _, point = cv2.minMaxLoc(probMap)

      # Scale the point to fit on the original image
      x = (self.frame_w * point[0]) / W
      y = (self.frame_h * point[1]) / H

      if prob > self.prob_threshold:
        self.points.append([int(x), int(y)])
      else:
        self.points.append([OVERFLOW_NULL, OVERFLOW_NULL])

  def detect(self):
    """
    Main operation to recognize body pose using a trained model

    :param frame: raw h264 decoded frame
    :return:
            draw_skeleton_flag: the flag that indicates if the skeleton
            are detected and depend if the skeleton is drawn on the pic
            pose: the command to be received by Tello
            points:the coordinates of the skeleton nodes
    """
    period_start_time = 0
    self.clear_detection_state()

    self.preprocess(self.current_frame)

    # get the output of the neural network and calculate the period of the process
    if self.period == 0:
      period_start_time = time.time()

    output = self.net.forward()

    if self.period == 0:
      period_end_time = time.time()

      # calculation the period of pose reconigtion for 6 times,and get the average value
      self.calculate_period(period_start_time, period_end_time)

      # set the frame_cnt_threshold and pose_captured_threshold according to
      # the period of the pose recognition
      self.set_detection_thresholds()

    # calculate the detected points
    self.handle_pose_points(output)

    # check the captured pose
    self.calculate_pose()

    # inc frame counter
    self.frame_cnt += 1

    # check whether pose control command are generated once for
    # certain times of pose recognition
    self.calculate_period_pose()
