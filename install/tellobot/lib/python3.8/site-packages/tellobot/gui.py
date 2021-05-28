import cv2

# +++++++++++++===============================
WINDOW = "ML SHIT"
POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11],
              [11, 12], [12, 13]]
CENTER_BOX_HALF_SIZE = 128
NORMAL_COLOR = (66, 144, 245)
ACTIVE_COLOR = (0, 0, 245)
overflow_null = -999

def draw_pose(frame, ai, points):
    pose_line_color = NORMAL_COLOR

    # highlight skeleton when there is a pose
    # if ai.current_pose != '':
    #     pose_line_color = ACTIVE_COLOR

    # Draw the detected skeleton points
    if len(points) != 0:
        for i in range(15):
            # Draw Skeleton
            for pair in POSE_PAIRS:
                partA = pair[0]
                partB = pair[1]

                if points[partA][0] != overflow_null and points[partA][1] != overflow_null and points[partB][0] != overflow_null and points[partB][1] != overflow_null:
                    cv2.line(
                        frame,
                        (points[partA][0], points[partA][1]),
                        (points[partB][0], points[partB][1]),
                        pose_line_color,
                        2)

            # Draw points
            cv2.circle(
                frame,
                points[i],
                8,
                NORMAL_COLOR,
                thickness=-1,
                lineType=cv2.FILLED)

            # Draw Labels
            cv2.putText(
                frame,
                "{}".format(i),
                points[i],
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                NORMAL_COLOR,
                1,
                lineType=cv2.LINE_AA)


class GUI:
    def __init__(self):
        self.window_width = None
        self.window_height = None

        self.center_box_points = None
        self.center_point = None

        cv2.namedWindow(WINDOW)
        cv2.moveWindow(WINDOW, 600, 360)  # do this dynamically
        cv2.resizeWindow(WINDOW, 600, 500)

    def set_window_size(self, frame):
        self.window_width = frame.shape[0]
        self.window_height = frame.shape[0]
        cv2.resizeWindow(WINDOW, self.window_width, self.window_height)

    def get_center_box_points(self, frame):
        w = frame.shape[0]
        h = frame.shape[1]

        self.center_point = (int(h / 2), int(w / 2))
        self.center_box_points = [
            (self.center_point[0] - CENTER_BOX_HALF_SIZE, self.center_point[1] - CENTER_BOX_HALF_SIZE),
            (self.center_point[0] + CENTER_BOX_HALF_SIZE, self.center_point[1] - CENTER_BOX_HALF_SIZE),
            (self.center_point[0] + CENTER_BOX_HALF_SIZE, self.center_point[1] + CENTER_BOX_HALF_SIZE),
            (self.center_point[0] - CENTER_BOX_HALF_SIZE, self.center_point[1] + CENTER_BOX_HALF_SIZE),
        ]

    def draw_info(self, frame, current_pose = '--', drone_cmd = '--'):
        # DRAW COMMAND
        cv2.putText(
            frame,
            'Detected pose: {0}'.format(current_pose),
            (16, 32),
            cv2.FONT_HERSHEY_PLAIN,
            1.75,
            NORMAL_COLOR,
            2)

        # DRAW DRONE CMD
        cv2.putText(
            frame,
            'Drone CMD: {0}'.format(drone_cmd),
            (16, 64),
            cv2.FONT_HERSHEY_PLAIN,
            1.75,
            NORMAL_COLOR,
            2)

    def draw_box(self, frame, ai):
        box_line_color = NORMAL_COLOR

        # # highlight box
        # if ai.is_pose_in_box:
        #     box_line_color = ACTIVE_COLOR

        print(self.center_point)
        # DRAW Center
        cv2.circle(
            frame,
            self.center_point,
            8,
            box_line_color,
            thickness=-1,
            lineType=cv2.FILLED)

        # change line color if person is in box
        if self.center_box_points is not None:
            cv2.line(
                frame,
                self.center_box_points[0],
                self.center_box_points[1],
                box_line_color,
                2)
            cv2.line(
                frame,
                self.center_box_points[1],
                self.center_box_points[2],
                box_line_color,
                2)
            cv2.line(
                frame,
                self.center_box_points[2],
                self.center_box_points[3],
                box_line_color,
                2)
            cv2.line(
                frame,
                self.center_box_points[3],
                self.center_box_points[0],
                box_line_color,
                2)

    def update_image(self, frame, points, ai, current_pose, drone_cmd):
        if frame is not None:
            draw_pose(frame, ai, points)
            self.draw_info(frame, current_pose, drone_cmd)
            self.draw_box(frame, ai)
            cv2.imshow(WINDOW, frame)
