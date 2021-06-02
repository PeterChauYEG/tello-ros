from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT
from tellobot.cmds import CMDS

class AI:
    def __init__(self):
        self.current_pose = ''
        self.is_pose_in_box = False
        self.distance = {
            "x": 0,
            "z": 0
        }
        self.drone_cmd = CMDS['NONE']
        self.find_human_tick = 0
        self.max_find_human_period = 0.5
        self.center_point = (TARGET_FRAME_WIDTH / 2, TARGET_FRAME_HEIGHT / 2)
        self.user_cmd = CMDS['NONE']
        self.center_box_half_size = int(TARGET_FRAME_HEIGHT / 4)
        self.pose_centered_sensitivity = 0

    def reset_state(self):
        self.current_pose = ''
        self.is_pose_in_box = False
        self.drone_cmd = CMDS['NONE']
        self.user_cmd = CMDS['NONE']

    def get_center_human_cmd(self):
        # when the human is not centered
        if self.is_pose_in_box is False:

            # when we can't find the human
            # go back and forth
            if self.current_pose == '':
                self.find_human_tick += 1

                if self.find_human_tick > self.max_find_human_period:
                    self.distance['x'] *= -1
                    self.distance['z'] *= -1
                    self.find_human_tick = 0

            # determine if x or y is more off
            abs_distance = {
                'x': abs(self.distance['x']),
                'z': abs(self.distance['z']),
            }

            if abs_distance['x'] > abs_distance['z']:
                if self.distance['x'] < 0:
                    self.drone_cmd = CMDS['X_DEC']
                elif self.distance['x'] > 0:
                    self.drone_cmd = CMDS['X_INC']
            else:
                if self.distance['z'] < 0:
                    self.drone_cmd = CMDS['Z_DEC']
                elif self.distance['z'] > 0:
                    self.drone_cmd = CMDS['Z_INC']

    def update_current_pose(self, current_pose):
        self.current_pose = current_pose

    def update_user_cmd(self, user_cmd):
        self.user_cmd = user_cmd
        self.drone_cmd = user_cmd

    def calculate_head_in_box(self, pose_points):
        key_pose_points = [pose_points[0], pose_points[1]]
        self.get_sum_of_distance(key_pose_points)
        self.get_is_pose_in_box()

        if self.user_cmd == CMDS['NONE']:
            self.get_center_human_cmd()

    def get_sum_of_distance(self, points):
        self.distance = {
            "x": 0,
            "z": 0
        }

        if self.center_point is not None and points is not None:
            for point in points:
                if point is not None:
                    if point[0] < self.center_point[0] - self.center_box_half_size:
                        self.distance['x'] = self.distance['x'] + (point[0] - (self.center_point[0] - self.center_box_half_size))
                    elif point[0] > self.center_point[0] + self.center_box_half_size:
                        self.distance['x'] = self.distance['x'] + (point[0] - (self.center_point[0] + self.center_box_half_size))

                    if point[1] < self.center_point[1] - self.center_box_half_size:
                        self.distance['z'] = self.distance['z'] + (point[1] - (self.center_point[1] - self.center_box_half_size))
                    elif point[1] > self.center_point[1] + self.center_box_half_size:
                        self.distance['z'] = self.distance['z'] + (point[1] - (self.center_point[1] + self.center_box_half_size))

    def get_is_pose_in_box(self):
        if self.pose_centered_sensitivity >= self.distance[
            'x'] >= -self.pose_centered_sensitivity and self.pose_centered_sensitivity >= \
                self.distance['z'] >= -self.pose_centered_sensitivity:
            self.is_pose_in_box = True
            return

        self.is_pose_in_box = False
