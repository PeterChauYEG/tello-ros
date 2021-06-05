import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tellobot_interfaces.msg import DroneInfo
from tellobot.cmds import CMDS, TELLO_CMDS
from tellobot.fake_drone import FakeDrone
from tellobot.tello_drone import TelloDrone


class DroneNode(Node):
  def __init__(self):
    super().__init__('drone_node')

    timer_period = 10

    self.drone = None

    self.declare_parameter('drone_type', 'fake_drone')
    self.handle_drone_type()

    self.drone_cmd_subscription = self.create_subscription(
      String,
      'drone_cmd',
      self.listener_drone_cmd_callback,
      1)

    self.drone_info_publisher = self.create_publisher(DroneInfo, 'drone_info', 500)

    self.timer = self.create_timer(timer_period, self.timer_callback)

  def create_drone_info_msg(self, battery, speed):
    msg = DroneInfo()
    msg.battery = battery
    msg.speed = speed
    return msg

  def convert_drone_cmd_to_tello_cmd(self, drone_cmd):
    tello_cmd = TELLO_CMDS[drone_cmd]

    if drone_cmd not in (CMDS['TAKE_OFF'], CMDS['LAND'], CMDS['NONE']):
      tello_cmd = tello_cmd + ' %d' % 20

    return tello_cmd

  def handle_drone_type(self):
    drone_type = self.get_parameter('drone_type').get_parameter_value().string_value

    if self.drone:
      self.drone.stop()

    if drone_type == 'fake_drone':
      self.drone = FakeDrone()
    else:
      self.drone = TelloDrone()

    self.drone.start()

  def timer_callback(self):
    drone_battery = self.drone.get_battery()
    drone_speed = self.drone.get_speed()

    print('s %s' % drone_speed)
    print('b %s' % drone_battery)
    if drone_speed and drone_speed:
      self.drone_info_publisher.publish(self.create_drone_info_msg(drone_battery, drone_speed))

  def listener_drone_cmd_callback(self, msg):
    tello_cmd = self.convert_drone_cmd_to_tello_cmd(msg.data)

    if tello_cmd != CMDS['NONE']:
      if tello_cmd == CMDS['TAKE_OFF']:
        self.drone.takeoff()
      if tello_cmd == CMDS['LAND']:
        self.drone.land()
      else:
        self.drone.send_command(tello_cmd)


def main(args=None):
  rclpy.init(args=args)
  drone_node = DroneNode()

  rclpy.spin(drone_node)

  drone_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
