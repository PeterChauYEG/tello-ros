import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Bool, UInt8MultiArray
from tellobot_interfaces.msg import StringArray
from tellobot.cmds import CMDS
from tellobot.gui_camera import GUICamera
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT


class GUICameraNode(Node):
  def __init__(self):
    super().__init__('gui_camera_node')

    self.pose = '--'
    self.pose_points = []
    self.drone_cmd = CMDS['NONE']
    self.is_pose_in_box = False
    self.detected_objects = []
    self.detected_object_labels = []

    self.gui = GUICamera()

    self.video_frame_subscription = self.create_subscription(
      UInt8MultiArray,
      'video_frames',
      self.listener_video_frames_callback,
      1)

    self.pose_subscription = self.create_subscription(
      String,
      'pose',
      self.listener_pose_callback,
      1)

    self.is_pose_in_box_subscription = self.create_subscription(
      Bool,
      'is_pose_in_box',
      self.listener_is_pose_in_box_callback,
      1)

    self.pose_points_subscription = self.create_subscription(
      Int16MultiArray,
      'pose_points',
      self.listener_pose_points_callback,
      1)

    self.detected_object_labels_subscription = self.create_subscription(
      StringArray,
      'detected_object_labels',
      self.detected_object_labels_callback,
      1)

    self.detected_objects_subscription = self.create_subscription(
      Int16MultiArray,
      'detected_objects',
      self.detected_objects_callback,
      1)

  def convert_ros_msg_to_frame(self, msg):
    current_frame = np.array(msg.data)

    if len(current_frame) != TARGET_FRAME_HEIGHT * TARGET_FRAME_WIDTH * 3:
      return None

    # pylint: disable=too-many-function-args
    reshaped_frame = current_frame.reshape(TARGET_FRAME_HEIGHT, TARGET_FRAME_WIDTH, 3)

    return reshaped_frame

  def listener_pose_callback(self, msg):
    self.pose = msg.data

  def listener_is_pose_in_box_callback(self, msg):
    self.is_pose_in_box = msg.data

  def listener_pose_points_callback(self, msg):
    pose_points = np.array(msg.data)

    if pose_points.shape[0] != 30:
      return

    resized_frame = pose_points.reshape(15, 2).tolist()
    self.pose_points = resized_frame

  def detected_objects_callback(self, msg):
    objects = np.array(msg.data)

    resized_frame = objects.reshape(-1, 5).tolist()
    self.detected_objects = resized_frame

  def detected_object_labels_callback(self, msg):
    self.detected_object_labels = msg.data

  def listener_video_frames_callback(self, msg):
    resized_frame = self.convert_ros_msg_to_frame(msg)
    self.gui.update_image(
      resized_frame,
      self.pose_points,
      self.pose,
      self.is_pose_in_box,
      self.detected_objects,
      self.detected_object_labels)

    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)

  gui_camera_node = GUICameraNode()

  rclpy.spin(gui_camera_node)
  gui_camera_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
