import cv2
from tellobot.web_camera import WINDOW_WIDTH, WINDOW_HEIGHT

# +++++++++++++===============================
WINDOW = "ML SHIT"
POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11],
              [11, 12], [12, 13]]
CENTER_BOX_HALF_SIZE = 128
NORMAL_COLOR = (66, 144, 245)
ACTIVE_COLOR = (0, 0, 245)
overflow_null = -999


def draw_pose(frame, current_pose, points):
    pose_line_color = NORMAL_COLOR

    # highlight skeleton when there is a pose
    if current_pose != '':
        pose_line_color = ACTIVE_COLOR

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


class GUICamera:
    def __init__(self):
        self.center_point = (int(WINDOW_WIDTH / 2), int(WINDOW_HEIGHT / 2))
        self.center_box_points = self.get_center_box_points()

        cv2.namedWindow(WINDOW)
        cv2.moveWindow(WINDOW, 600, 360)  # do this dynamically

    def get_center_box_points(self):
        return [
            (self.center_point[0] - CENTER_BOX_HALF_SIZE, self.center_point[1] - CENTER_BOX_HALF_SIZE),
            (self.center_point[0] + CENTER_BOX_HALF_SIZE, self.center_point[1] - CENTER_BOX_HALF_SIZE),
            (self.center_point[0] + CENTER_BOX_HALF_SIZE, self.center_point[1] + CENTER_BOX_HALF_SIZE),
            (self.center_point[0] - CENTER_BOX_HALF_SIZE, self.center_point[1] + CENTER_BOX_HALF_SIZE),
        ]

    def draw_box(self, frame, is_pose_in_box):
        box_line_color = NORMAL_COLOR

        # highlight box
        if is_pose_in_box:
            box_line_color = ACTIVE_COLOR

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

    def update_image(self, frame, points, current_pose, is_pose_in_box):
        if frame is not None:
            draw_pose(frame, current_pose, points)
            self.draw_box(frame, is_pose_in_box)
            cv2.imshow(WINDOW, frame)
