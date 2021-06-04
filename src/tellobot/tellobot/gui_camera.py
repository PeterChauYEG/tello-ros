import cv2
from tellobot.ai_constants import POSE_PAIRS, OVERFLOW_NULL
from tellobot.colors import POSE_COLOR, POSE_IN_BOX_COLOR, BOX_COLOR, \
  POSE_DETECTED_COLOR, IMAGE_TEXT_COLOR
from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT, WINDOW


def draw_pose(frame, current_pose, points):
  pose_line_color = POSE_COLOR

  # highlight skeleton when there is a pose
  if current_pose != '':
    pose_line_color = POSE_DETECTED_COLOR

  # Draw the detected skeleton points
  if len(points) != 0:
    # Draw Skeleton
    for pair in POSE_PAIRS:
      partA = pair[0]
      partB = pair[1]

      if OVERFLOW_NULL not in (points[partA][0], points[partA][1], points[partB][0], points[partB][1]):
        cv2.line(
          frame,
          (points[partA][0], points[partA][1]),
          (points[partB][0], points[partB][1]),
          pose_line_color,
          2)

    for i in range(15):
      # Draw points
      cv2.circle(
        frame,
        points[i],
        4,
        POSE_COLOR,
        thickness=-1,
        lineType=cv2.FILLED)

      # Draw Labels
      cv2.putText(
        frame,
        "{}".format(i),
        points[i],
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        IMAGE_TEXT_COLOR,
        2,
        lineType=cv2.LINE_AA)


class GUICamera:
  def __init__(self):
    self.center_box_points = self.get_center_box_points()

    cv2.namedWindow(WINDOW)
    cv2.moveWindow(WINDOW, 600, 360)  # do this dynamically

  def get_center_box_points(self):
    return [
      (CENTER_POINT[0] - GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] - GUI_CENTER_BOX_HALF_SIZE),
      (CENTER_POINT[0] + GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] - GUI_CENTER_BOX_HALF_SIZE),
      (CENTER_POINT[0] + GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] + GUI_CENTER_BOX_HALF_SIZE),
      (CENTER_POINT[0] - GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] + GUI_CENTER_BOX_HALF_SIZE),
    ]

  def draw_box(self, frame, is_pose_in_box):
    box_line_color = BOX_COLOR

    # highlight box
    if is_pose_in_box:
      box_line_color = POSE_IN_BOX_COLOR

    # DRAW Center
    cv2.circle(
      frame,
      CENTER_POINT,
      4,
      POSE_IN_BOX_COLOR,
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
