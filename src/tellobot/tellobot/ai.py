from threading import Thread

# pylint: disable=line-too-long
from tellobot.ai_constants import MAX_HEAD_Y, MIN_HEAD_Y, OVERFLOW_NULL, DEFAULT_DRONE_MOVE_DISTANCE, \
  DEFAULT_DRONE_MOVE_DEGREES, POSE_CENTERED_SENSITIVITY, SENTRY_SEQUENCE
from tellobot.cmds import CMDS, DRONE_CMDS
from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT


class AI:
  def __init__(self):
    self.user_cmd = CMDS['NONE']
    self.drone_cmd = CMDS['NONE']
    self.mode = CMDS['USER']
    self.taken_off = False
    self.thread_started = False

    self.pose_points = None
    self.current_pose = ''
    self.is_pose_in_box = False
    self.distance = {
      "x": 0,
      "z": 0,
      "y": 0
    }

    self.sentry_sequence_index = 0

    self.find_human_tick = 0
    self.max_find_human_period = 0.5

    self.thread = Thread(target=self.update, args=(), daemon=True)
    self.start()

  def reset_state(self):
    self.current_pose = ''
    self.drone_cmd = CMDS['NONE']
    self.user_cmd = CMDS['NONE']
    self.pose_points = None

  def get_drone_cmd(self):

    if self.mode == CMDS['SENTRY']:
      self.sentry_update()
      return self.drone_cmd

    cmd = self.drone_cmd

    if self.mode == CMDS['USER']:
      self.user_control_update()

    return cmd

  def update_pose_points(self, pose_points):
    self.pose_points = pose_points

  def update_current_pose(self, current_pose):
    self.current_pose = current_pose

  def update_user_cmd_and_drone_cmd(self, user_cmd):
    self.reset_state()
    self.user_cmd = user_cmd

    if user_cmd == CMDS['TAKE_OFF']:
      self.taken_off = True

    if self.taken_off is True or user_cmd == CMDS['SENTRY']:
      self.calculate_drone_cmd_from_cmd(user_cmd, None)

    if user_cmd == CMDS['LAND']:
      self.taken_off = False

  def calculate_drone_cmd_from_cmd(self, cmd, amount):
    if cmd == CMDS['NONE']:
      self.drone_cmd = cmd

    if cmd == CMDS['TRACK']:
      self.mode = cmd

    if cmd == CMDS['SENTRY']:
      self.mode = cmd

    if cmd == CMDS['USER']:
      self.mode = cmd

    tello_cmd = DRONE_CMDS[cmd]

    if cmd in (
        CMDS['X_DEC'],
        CMDS['X_INC'],
        CMDS['Y_DEC'],
        CMDS['Y_INC'],
        CMDS['Z_INC'],
        CMDS['Z_DEC']):
      cmd_amount = DEFAULT_DRONE_MOVE_DISTANCE

      if amount:
        cmd_amount = amount

      self.drone_cmd = '{0} {1}'.format(tello_cmd, cmd_amount)

    if cmd in (CMDS['Z_CW'], CMDS['Z_CCW']):
      cmd_amount = DEFAULT_DRONE_MOVE_DEGREES

      if amount:
        cmd_amount = amount

      if self.mode == CMDS['SENTRY']:
        cmd_amount = 30

      self.drone_cmd = '{0} {1}'.format(tello_cmd, cmd_amount)

    if cmd in (
        CMDS['FLIP_L'],
        CMDS['FLIP_R'],
        CMDS['FLIP_F'],
        CMDS['FLIP_B'],
        CMDS['FLIP_BL'],
        CMDS['FLIP_BR'],
        CMDS['FLIP_FL'],
        CMDS['FLIP_FR'],
        CMDS['TAKE_OFF'],
        CMDS['LAND']):
      self.drone_cmd = tello_cmd

  def start(self):
    self.thread_started = True
    self.thread.start()

  def stop(self):
    self.thread_started = False

  def update(self):
    while True:
      if not self.thread_started:
        return

      if self.user_cmd != CMDS['NONE'] or self.taken_off is False:
        self.reset_state()

      if self.mode == CMDS['TRACK']:
        self.track_update()

  def track_update(self):
    if not self.pose_points:
      return

    key_pose_points = [self.pose_points[0], self.pose_points[1]]
    self.get_sum_of_distance(key_pose_points)
    self.get_is_pose_in_box()

    if self.is_pose_in_box:
      self.reset_state()
      return

    cmd = self.get_center_human_cmd()

    if cmd is None:
      self.reset_state()
      return

    self.calculate_drone_cmd_from_cmd(cmd, None)

  def sentry_update(self):
    if self.sentry_sequence_index >= len(SENTRY_SEQUENCE):
      self.mode = CMDS['USER']
      self.reset_state()
      self.sentry_sequence_index = 0
      return

    if self.sentry_sequence_index == 0:
      self.taken_off = True

    cmd = SENTRY_SEQUENCE[self.sentry_sequence_index]
    self.update_user_cmd_and_drone_cmd(cmd)
    self.sentry_sequence_index += 1

  def user_control_update(self):
    self.reset_state()

  # tracking methods
  def get_center_human_cmd(self):
    # determine if x or y is more off
    abs_distance = {
      'x': abs(self.distance['x']),
      'z': abs(self.distance['z']),
      'y': abs(self.distance['y']),
    }

    if abs_distance['x'] >= abs_distance['z'] and abs_distance['x'] >= abs_distance['y']:
      cmd = CMDS['Z_CW']

      if self.distance['x'] < 0:
        cmd = CMDS['Z_CCW']

      return cmd

    if abs_distance['z'] >= abs_distance['x'] and abs_distance['z'] >= abs_distance['y']:
      cmd = CMDS['Z_DEC']

      if self.distance['z'] < 0:
        cmd = CMDS['Z_INC']

      return cmd

    if abs_distance['y'] >= abs_distance['z'] and abs_distance['y'] >= abs_distance['x']:
      cmd = CMDS['Y_INC']

      if self.distance['y'] < 0:
        cmd = CMDS['Y_DEC']

      return cmd

    return None

  def get_sum_of_distance(self, points):
    self.distance = {
      "x": 0,
      "z": 0,
      "y": 0
    }

    if points is None:
      return

    if points[0] is not None and points[1] is not None and \
        points[0][1] != OVERFLOW_NULL and points[1][
      1] != OVERFLOW_NULL:
      head_size = points[1][1] - points[0][1]

      if head_size < MIN_HEAD_Y:
        self.distance['y'] = MIN_HEAD_Y - head_size
      elif head_size > MAX_HEAD_Y:
        self.distance['y'] = MAX_HEAD_Y - head_size

    if CENTER_POINT is not None:
      for point in points:
        if point is not None and point[0] != OVERFLOW_NULL and point[1] != OVERFLOW_NULL:
          if point[0] < CENTER_POINT[0] - GUI_CENTER_BOX_HALF_SIZE:
            self.distance['x'] = self.distance['x'] + (
                point[0] - (CENTER_POINT[0] - GUI_CENTER_BOX_HALF_SIZE))
          elif point[0] > CENTER_POINT[0] + GUI_CENTER_BOX_HALF_SIZE:
            self.distance['x'] = self.distance['x'] + (
                point[0] - (CENTER_POINT[0] + GUI_CENTER_BOX_HALF_SIZE))

          if point[1] < CENTER_POINT[1] - GUI_CENTER_BOX_HALF_SIZE:
            self.distance['z'] = self.distance['z'] + (
                point[1] - (CENTER_POINT[1] - GUI_CENTER_BOX_HALF_SIZE))
          elif point[1] > CENTER_POINT[1] + GUI_CENTER_BOX_HALF_SIZE:
            self.distance['z'] = self.distance['z'] + (
                point[1] - (CENTER_POINT[1] + GUI_CENTER_BOX_HALF_SIZE))

  def get_is_pose_in_box(self):
    if POSE_CENTERED_SENSITIVITY >= self.distance['x'] >= -POSE_CENTERED_SENSITIVITY and POSE_CENTERED_SENSITIVITY >= \
        self.distance['z'] >= -POSE_CENTERED_SENSITIVITY:
      self.is_pose_in_box = True
      return

    self.is_pose_in_box = False
