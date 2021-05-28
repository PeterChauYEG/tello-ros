CENTER_BOX_HALF_SIZE = 128
POSE_CENTERED_SENSITIVITY = 50


class AI:
    def __init__(self):
        self.current_pose = ''
        self.is_pose_in_box = False
        self.distance = {
            "x": 0,
            "y": 0
        }
        self.drone_cmd = ''
        self.find_human_tick = 0
        self.max_find_human_period = 0.5

    def reset_state(self):
        self.current_pose = ''
        self.is_pose_in_box = False
        self.drone_cmd = ''

    def get_center_human_cmd(self):
        # when the human is not centered
        if self.is_pose_in_box is False:

            # when we can't find the human
            # go back and forth
            if self.current_pose == '':
                self.find_human_tick += 1

                if self.find_human_tick > self.max_find_human_period:
                    self.distance['x'] *= -1
                    self.distance['y'] *= -1
                    self.find_human_tick = 0

            # determine if x or y is more off
            abs_distance = {
                'x': abs(self.distance['x']),
                'y': abs(self.distance['y']),
            }

            if abs_distance['x'] > abs_distance['y']:
                if self.distance['x'] < 0:
                    self.drone_cmd = 'x-dec'
                elif self.distance['x'] > 0:
                    self.drone_cmd = 'x-inc'
            else:
                if self.distance['y'] < 0:
                    self.drone_cmd = 'y-inc'
                elif self.distance['y'] > 0:
                    self.drone_cmd = 'y-dec'

    def update_current_pose(self, current_pose):
        self.current_pose = current_pose

    def get_sum_of_distance(self, points, center_point):
        if center_point is not None and points is not None:
            for point in points:
                if point is not None:
                    self.distance = {
                        "x": 0,
                        "y": 0
                    }

                    if point[0] < center_point[0] - CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (point[0] - (center_point[0] - CENTER_BOX_HALF_SIZE))
                    elif point[0] > center_point[0] + CENTER_BOX_HALF_SIZE:
                        self.distance['x'] = self.distance['x'] + (point[0] - (center_point[0] + CENTER_BOX_HALF_SIZE))

                    if point[1] < center_point[1] - CENTER_BOX_HALF_SIZE:
                        self.distance['y'] = self.distance['y'] + (point[1] - (center_point[1] - CENTER_BOX_HALF_SIZE))
                    elif point[1] > center_point[1] + CENTER_BOX_HALF_SIZE:
                        self.distance['y'] = self.distance['y'] + (point[1] - (center_point[1] + CENTER_BOX_HALF_SIZE))

    def get_is_pose_in_box(self):
        if POSE_CENTERED_SENSITIVITY > self.distance[
            'x'] > -POSE_CENTERED_SENSITIVITY and POSE_CENTERED_SENSITIVITY > \
                self.distance['y'] > -POSE_CENTERED_SENSITIVITY:
            self.is_pose_in_box = True

        self.is_pose_in_box = False
