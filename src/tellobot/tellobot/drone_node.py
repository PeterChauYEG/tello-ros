import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

from tellobot.fake_drone import FakeDrone
from tellobot.tello_drone import TelloDrone
from tellobot.cmds import CMDS, TELLO_CMDS

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
            10)

        self.drone_battery_publisher = self.create_publisher(Int8, 'drone_battery', 10)
        self.drone_speed_publisher = self.create_publisher(Int8, 'drone_speed', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_to_float_ros_msg(self, data):
        msg = Int8()
        msg.data = data
        return msg

    def create_none_user_cmd_ros_msg(self, user_cmd):
        msg = String()
        msg.data = user_cmd
        return msg

    def convert_drone_cmd_to_tello_cmd(self, drone_cmd):
        tello_cmd = TELLO_CMDS[drone_cmd]

        if drone_cmd != CMDS['TAKE_OFF'] and drone_cmd != CMDS['LAND'] and drone_cmd != CMDS['NONE']:
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

        self.drone_battery_publisher.publish(self.create_to_float_ros_msg(drone_battery))
        self.drone_speed_publisher.publish(self.create_to_float_ros_msg(drone_speed))

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
