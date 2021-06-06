import cv2
from tellobot.ai_constants import POSE_PAIRS, OVERFLOW_NULL
from tellobot.colors import POSE_COLOR, POSE_IN_BOX_COLOR, BOX_COLOR, \
  POSE_DETECTED_COLOR, IMAGE_TEXT_COLOR
from tellobot.gui_constants import GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT, CAMERA_WINDOW
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT

class GUICamera:
  def __init__(self):
    self.center_box_points = self.get_center_box_points()

    cv2.namedWindow(CAMERA_WINDOW)
    cv2.moveWindow(CAMERA_WINDOW, 600, 360)  # do this dynamically
    cv2.setMouseCallback(CAMERA_WINDOW, self.click_and_crop)
    self.frame = []
    self.clicked_point = None

  def draw_pose(self, current_pose, points):
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

        # pylint: disable=line-too-long
        if OVERFLOW_NULL not in (
            points[partA][0],
            points[partA][1],
            points[partB][0],
            points[partB][1]):
          cv2.line(
            self.frame,
            (points[partA][0], points[partA][1]),
            (points[partB][0], points[partB][1]),
            pose_line_color,
            2)

      for i in range(15):
        # Draw points
        cv2.circle(
          self.frame,
          points[i],
          4,
          POSE_COLOR,
          thickness=-1,
          lineType=cv2.FILLED)

        # Draw Labels
        cv2.putText(
          self.frame,
          "{}".format(i),
          points[i],
          cv2.FONT_HERSHEY_SIMPLEX,
          0.75,
          IMAGE_TEXT_COLOR,
          2,
          lineType=cv2.LINE_AA)

  def get_center_box_points(self):
    return [
      (CENTER_POINT[0] - GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] - GUI_CENTER_BOX_HALF_SIZE),
      (CENTER_POINT[0] + GUI_CENTER_BOX_HALF_SIZE, CENTER_POINT[1] + GUI_CENTER_BOX_HALF_SIZE),
    ]

  def draw_box(self, is_pose_in_box):
    box_line_color = BOX_COLOR
    line_size = 2

    # highlight box
    if is_pose_in_box:
      box_line_color = POSE_IN_BOX_COLOR
      line_size = 3

    # DRAW Center
    cv2.circle(
      self.frame,
      CENTER_POINT,
      4,
      POSE_IN_BOX_COLOR,
      thickness=-1,
      lineType=cv2.FILLED)

    # change line color if person is in box
    if self.center_box_points is not None:
      cv2.rectangle(
        self.frame,
        self.center_box_points[0],
        self.center_box_points[1],
        box_line_color,
        line_size)

  def draw_detected_object_boxes(self, detected_objects, detected_object_labels):
    box_line_color = BOX_COLOR

    if len(detected_objects) != 0:

      for i, detected_object in enumerate(detected_objects):
        line_size = 1

        if self.clicked_point is not None and detected_object[3] > self.clicked_point[0] > detected_object[1] and detected_object[4] > self.clicked_point[1] > detected_object[2]:
          line_size = 2

        cv2.rectangle(
          self.frame,
          (detected_object[1], detected_object[2]),
          (detected_object[3], detected_object[4]),
          box_line_color,
          line_size)
        cv2.putText(
          self.frame,
          "{0} {1}%".format(detected_object_labels[i], detected_object[0]),
          (detected_object[1], detected_object[2]),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          IMAGE_TEXT_COLOR,
          1,
          lineType=cv2.LINE_AA)
  def draw_detected_object_boxes(self, detected_objects, detected_object_labels):
    box_line_color = BOX_COLOR

    if len(detected_objects) != 0:

      for i, detected_object in enumerate(detected_objects):
        line_size = 1
        label_point = (detected_object[1], detected_object[2] + 16)

        if self.clicked_point is not None and detected_object[3] > self.clicked_point[0] > detected_object[1] and detected_object[4] > self.clicked_point[1] > detected_object[2]:
          line_size = 2

        if detected_object[1] < 0:
          label_point = (detected_object[3] - 150, label_point[1])

        if detected_object[2] < 0:
          label_point = (label_point[0], detected_object[4] - 10)

        if detected_object[1] < 0 and detected_object[3] > TARGET_FRAME_WIDTH and detected_object[2] < 0 and detected_object[4] > TARGET_FRAME_HEIGHT:
          width = (detected_object[2] - detected_object[1]) / 2
          height =( detected_object[4] - detected_object[3]) /2
          label_point = (label_point[0] + (width - 75), label_point[1] + (height - 10))

        cv2.rectangle(
          self.frame,
          (detected_object[1], detected_object[2]),
          (detected_object[3], detected_object[4]),
          box_line_color,
          line_size)
        cv2.putText(
          self.frame,
          "{0} {1}%".format(detected_object_labels[i], detected_object[0]),
          label_point,
          cv2.FONT_HERSHEY_SIMPLEX,
          0.75,
          IMAGE_TEXT_COLOR,
          2,
          lineType=cv2.LINE_AA)

  def draw_clicked_point(self):
    if self.clicked_point is not None:
      cv2.circle(
        self.frame,
        self.clicked_point,
        4,
        POSE_DETECTED_COLOR,
        thickness=-1,
        lineType=cv2.FILLED)

  def click_and_crop(self, event, x, y, flags, param):
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONUP:
      self.clicked_point = (x, y)

    if event == cv2.EVENT_RBUTTONUP:
      self.clicked_point = None

  # pylint: disable=line-too-long
  def update_image(self, frame, points, current_pose, is_pose_in_box, detected_objects, detected_object_labels):
    if frame is not None:
      self.frame = frame
      self.draw_pose(current_pose, points)
      self.draw_box(is_pose_in_box)
      self.draw_detected_object_boxes(detected_objects, detected_object_labels)
      self.draw_clicked_point()
      cv2.imshow(CAMERA_WINDOW, self.frame)
