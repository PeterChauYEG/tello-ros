from threading import Thread

from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT
from tellobot.cmds import CMDS
from tellobot.ai_constants import MAX_HEAD_Y, MIN_HEAD_Y, POSE_CENTERED_SENSITIVITY, OVERFLOW_NULL


class AI:
    def __init__(self):
        self.current_pose = ''
        self.is_pose_in_box = False
        self.distance = {
            "x": 0,
            "z": 0,
            "y": 0
        }
        self.drone_cmd = CMDS['NONE']
        self.find_human_tick = 0
        self.max_find_human_period = 0.5
        self.user_cmd = CMDS['NONE']
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
                'y': abs(self.distance['y']),
            }

            if abs_distance['x'] >= abs_distance['z'] and abs_distance['x'] >= abs_distance['y']:
                if self.distance['x'] < 0:
                    self.drone_cmd = CMDS['Z_CW']
                elif self.distance['x'] > 0:
                    self.drone_cmd = CMDS['Z_CCW']
            elif abs_distance['z'] >= abs_distance['x'] and abs_distance['z'] >= abs_distance['y']:
                if self.distance['z'] < 0:
                    self.drone_cmd = CMDS['Z_DEC']
                elif self.distance['z'] > 0:
                    self.drone_cmd = CMDS['Z_INC']
            elif abs_distance['y'] >= abs_distance['z'] and abs_distance['y'] >= abs_distance['x']:
                if self.distance['y'] < 0:
                    self.drone_cmd = CMDS['Y_DEC']
                elif self.distance['y'] > 0:
                    self.drone_cmd = CMDS['Y_INC']

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
            "z": 0,
            "y": 0
        }

        if points is None:
            return

        if points[0] is not None and points[1] is not None and points[0][1] != OVERFLOW_NULL and points[1][1] != OVERFLOW_NULL:
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
        if POSE_CENTERED_SENSITIVITY >= self.distance[
            'x'] >= -POSE_CENTERED_SENSITIVITY and POSE_CENTERED_SENSITIVITY >= \
                self.distance['z'] >= -POSE_CENTERED_SENSITIVITY:
            self.is_pose_in_box = True
            return

        self.is_pose_in_box = False
