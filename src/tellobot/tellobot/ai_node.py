import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, Bool
from tellobot_interfaces.srv import TakePicture, UserCmd
from tellobot.ai import AI


class AINode(Node):
  def __init__(self):
    super().__init__('ai_node')

    self.ai = AI()

    self.drone_cmd_publisher = self.create_publisher(String, 'drone_cmd', 10)
    self.is_pose_in_box_publisher = self.create_publisher(Bool, 'is_pose_in_box', 10)

    self.pose_subscription = self.create_subscription(
      String,
      'pose',
      self.listener_pose_callback,
      10)

    self.pose_points_subscription = self.create_subscription(
      Int16MultiArray,
      'pose_points',
      self.listener_pose_points_callback,
      10)

    self.take_picture_service = self.create_client(TakePicture, 'take_picture')
    self.user_cmd = self.create_service(UserCmd, 'user_cmd', self.user_cmd_callback)

    while not self.take_picture_service.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

  def send_take_picture_request(self):
    req = TakePicture.Request()
    req.trigger = True
    self.take_picture_service.call_async(req)

  def convert_drone_cmd_to_ros_msg(self, drone_cmd):
    msg = String()
    msg.data = drone_cmd
    return msg

  def convert_is_pose_in_box_to_ros_msg(self, is_pose_in_box):
    msg = Bool()
    msg.data = is_pose_in_box
    return msg

  def user_cmd_callback(self, request, response):
    self.ai.update_user_cmd_and_drone_cmd(request.cmd)
    drone_cmd = self.ai.drone_cmd
    self.drone_cmd_publisher.publish(self.convert_drone_cmd_to_ros_msg(drone_cmd))

    response.result = True
    return response

  def listener_pose_callback(self, msg):
    self.ai.update_current_pose(msg.data)

  def listener_pose_points_callback(self, msg):
    pose_points = np.array(msg.data)

    if pose_points.shape[0] != 30:
      return

    resized_pose_points = pose_points.reshape(15, 2).tolist()
    self.ai.update_pose_points(resized_pose_points)
    drone_cmd = self.ai.drone_cmd
    is_pose_in_box = self.ai.is_pose_in_box

    if is_pose_in_box:
      self.send_take_picture_request()

    self.is_pose_in_box_publisher.publish(self.convert_is_pose_in_box_to_ros_msg(is_pose_in_box))
    self.get_logger().info('%s' % drone_cmd)
    self.drone_cmd_publisher.publish(self.convert_drone_cmd_to_ros_msg(drone_cmd))


def main(args=None):
  rclpy.init(args=args)

  ai_node = AINode()

  rclpy.spin(ai_node)
  ai_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
