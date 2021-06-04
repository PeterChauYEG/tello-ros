import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Int16MultiArray
from tellobot_interfaces.msg import StringArray
from tellobot.object_detection import ObjectDetection
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT


class ObjectDetectionNode(Node):
  def __init__(self):
    super().__init__('object_detection_node')

    self.object_detection = ObjectDetection()

    self.detected_objects_publisher = self.create_publisher(Int16MultiArray, 'detected_objects', 1)
    self.detected_object_labels_publisher = self.create_publisher(StringArray, 'detected_object_labels', 1)
    self.subscription = self.create_subscription(
      UInt8MultiArray,
      'video_frames',
      self.listener_callback,
      1)

  def convert_objects_to_ros_msg(self, objects):
    msg = Int16MultiArray()
    msg.data = np.array(objects).reshape(-1).tolist()
    return msg

  def convert_object_labels_to_ros_msg(self, object_labels):
    msg = StringArray()
    msg.data = object_labels
    return msg

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
      self.detected_objects_publisher.publish(self.convert_objects_to_ros_msg(objects))
      self.detected_object_labels_publisher.publish(self.convert_object_labels_to_ros_msg(object_labels))

def main(args=None):
  rclpy.init(args=args)

  object_detection_node = ObjectDetectionNode()

  rclpy.spin(object_detection_node)

  object_detection_node.object_detection.stop()
  object_detection_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
