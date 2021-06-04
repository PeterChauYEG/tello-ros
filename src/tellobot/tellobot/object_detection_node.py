import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from tellobot.object_detection import ObjectDetection
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT


class ObjectDetectionNode(Node):
  def __init__(self):
    super().__init__('object_detection_node')

    self.object_detection = ObjectDetection()

    self.subscription = self.create_subscription(
      UInt8MultiArray,
      'video_frames',
      self.listener_callback,
      1)

  def convert_ros_msg_to_frame(self, msg):
    current_frame = np.array(msg.data)

    if len(current_frame) != TARGET_FRAME_HEIGHT * TARGET_FRAME_WIDTH * 3:
      return None

    # pylint: disable=too-many-function-args
    reshaped_frame = current_frame.reshape(TARGET_FRAME_HEIGHT, TARGET_FRAME_WIDTH, 3)
    return reshaped_frame

  def listener_callback(self, msg):
    resized_frame = self.convert_ros_msg_to_frame(msg)

    self.object_detection.set_current_frame(resized_frame)
    objects, object_labels = self.object_detection.read()

    if objects is not None:
      self.get_logger().info(object_labels[0])

# if pose is not None:
    #   msg = self.convert_pose_to_ros_msg(pose)
    #   self.pose_publisher.publish(msg)
    #
    # if len(pose_points) != 0:
    #   msg = self.convert_pose_points_to_ros_msg(pose_points)
    #   self.pose_points_publisher.publish(msg)


def main(args=None):
  rclpy.init(args=args)

  object_detection_node = ObjectDetectionNode()

  rclpy.spin(object_detection_node)

  object_detection_node.object_detection.stop()
  object_detection_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
