import cv2
import rclpy
from rclpy.node import Node
from tellobot_interfaces.msg import DroneInfo
from tellobot_interfaces.srv import UserCmd
from std_msgs.msg import String, Float32
from tellobot.gui_buttons import GUIButtons


class GUIButtonsNode(Node):
  def __init__(self):
    super().__init__('gui_buttons_node')

    self.drone_info_subscription = self.create_subscription(
      DroneInfo,
      'drone_battery',
      self.drone_info_callback,
      500)

    self.fps_subscription = self.create_subscription(
      Float32,
      'fps',
      self.fps_callback,
      500)

    self.drone_cmd_subscription = self.create_subscription(
      String,
      'drone_cmd',
      self.listener_drone_cmd_callback,
      1)

    self.pose_subscription = self.create_subscription(
      String,
      'pose',
      self.listener_pose_callback,
      1)

    self.user_cmd_service = self.create_client(UserCmd, 'user_cmd')

    while not self.user_cmd_service.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.gui_buttons = GUIButtons()

    timer_period = 0.01
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def send_user_cmd_request(self, cmd):
    req = UserCmd.Request()
    req.cmd = cmd
    self.user_cmd_service.call_async(req)

  def fps_callback(self, msg):
    self.gui_buttons.cam_fps = round(msg.data, 2)

  def drone_info_callback(self, msg):
    self.gui_buttons.drone_battery = msg.battery
    self.gui_buttons.drone_speed = msg.speed

  def listener_drone_cmd_callback(self, msg):
    self.gui_buttons.drone_cmd = msg.data

  def listener_pose_callback(self, msg):
    self.gui_buttons.pose = msg.data

  def timer_callback(self):
    self.gui_buttons.show(self.send_user_cmd_request)
    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)

  gui_buttons_node = GUIButtonsNode()

  rclpy.spin(gui_buttons_node)
  gui_buttons_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
