from tellobot.gui import WINDOW_WIDTH, WINDOW_HEIGHT
from tellobot.cmds import CMDS

CENTER_BOX_HALF_SIZE = 128
POSE_CENTERED_SENSITIVITY = 50


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
        self.center_point = (WINDOW_HEIGHT / 2, WINDOW_WIDTH / 2)
        self.user_cmd = CMDS['NONE']

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

    def get_sum_of_distance(self, points):
        if self.center_point is not None and points is not None:
            for point in points:
                if point is not None:
                    self.distance = {
                        "x": 0,
                        "z": 0
                    }

                    if point[0] < self.center_point[0] - CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (point[0] - (self.center_point[0] - CENTER_BOX_HALF_SIZE))
                    elif point[0] > self.center_point[0] + CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (point[0] - (self.center_point[0] + CENTER_BOX_HALF_SIZE))

                    if point[1] < self.center_point[1] - CENTER_BOX_HALF_SIZE:
                        self.distance['z'] = self.distance['z'] + (point[1] - (self.center_point[1] - CENTER_BOX_HALF_SIZE))
                    elif point[1] > self.center_point[1] + CENTER_BOX_HALF_SIZE:
                        self.distance['z'] = self.distance['z'] + (point[1] - (self.center_point[1] + CENTER_BOX_HALF_SIZE))

    def get_is_pose_in_box(self):
        if POSE_CENTERED_SENSITIVITY > self.distance[
            'x'] > -POSE_CENTERED_SENSITIVITY and POSE_CENTERED_SENSITIVITY > \
                self.distance['z'] > -POSE_CENTERED_SENSITIVITY:
            self.is_pose_in_box = True

        self.is_pose_in_box = False
