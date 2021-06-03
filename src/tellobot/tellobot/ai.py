from threading import Thread

from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT
from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE
from tellobot.cmds import CMDS
from tellobot.pose_ml import overflow_null


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
        self.pose_centered_sensitivity = 0
        self.thread_started = False
        self.pose_points = None

        self.thread = Thread(target=self.update, args=(), daemon=True)
        self.start()

    def read_drone_cmd_and_is_pose_in_box(self):
        return self.drone_cmd, self.is_pose_in_box

    def set_pose_points(self, pose_points):
        self.pose_points = pose_points

    def start(self):
        self.thread_started = True
        self.thread.start()

    def update(self):
        while True:
            if not self.thread_started:
                return

            if self.pose_points is not None:
                self.calculate_head_in_box()

    def stop(self):
        self.thread_started = False

    def __del__(self):
        if self.thread:
            self.thread.stop()

    def reset_state(self):
        self.current_pose = ''
        self.drone_cmd = CMDS['NONE']
        self.user_cmd = CMDS['NONE']
        self.pose_points = None

    def get_center_human_cmd(self):
        # when the human is not centered
        if not self.is_pose_in_box:
            # determine if x or y is more off
            abs_distance = {
                'x': abs(self.distance['x']),
                'z': abs(self.distance['z']),
            }

            if abs_distance['x'] >= abs_distance['z']:
                if self.distance['x'] < 0:
                    self.drone_cmd = CMDS['Z_CW']
                elif self.distance['x'] > 0:
                    self.drone_cmd = CMDS['Z_CCW']
            else:
                if self.distance['z'] < 0:
                    self.drone_cmd = CMDS['Z_DEC']
                elif self.distance['z'] > 0:
                    self.drone_cmd = CMDS['Z_INC']
        else:
            self.reset_state()

    def update_current_pose(self, current_pose):
        self.current_pose = current_pose

    def update_user_cmd(self, user_cmd):
        self.user_cmd = user_cmd
        self.drone_cmd = user_cmd

    def calculate_head_in_box(self):
        key_pose_points = [self.pose_points[0], self.pose_points[1]]
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
                if point is not None and point[0] != overflow_null and point[1] != overflow_null:
                    if point[0] < self.center_point[0] - GUI_CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (
                                    point[0] - (self.center_point[0] - GUI_CENTER_BOX_HALF_SIZE))
                    elif point[0] > self.center_point[0] + GUI_CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (
                                    point[0] - (self.center_point[0] + GUI_CENTER_BOX_HALF_SIZE))

                    if point[1] < self.center_point[1] - GUI_CENTER_BOX_HALF_SIZE:
                        self.distance['z'] = self.distance['z'] + (
                                    point[1] - (self.center_point[1] - GUI_CENTER_BOX_HALF_SIZE))
                    elif point[1] > self.center_point[1] + GUI_CENTER_BOX_HALF_SIZE:
                        self.distance['z'] = self.distance['z'] + (
                                    point[1] - (self.center_point[1] + GUI_CENTER_BOX_HALF_SIZE))

    def get_is_pose_in_box(self):
        if self.pose_centered_sensitivity >= self.distance[
            'x'] >= -self.pose_centered_sensitivity and self.pose_centered_sensitivity >= \
                self.distance['z'] >= -self.pose_centered_sensitivity:
            self.is_pose_in_box = True
            return

        self.is_pose_in_box = False
