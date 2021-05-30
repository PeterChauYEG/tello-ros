import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray, Float32, Bool

from tellobot.fake_tello import FakeTello
from tellobot.cmds import CMDS, TELLO_CMDS

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        timer_period = 0.1

        self.fake_tello = FakeTello('fake_tello')
        self.tello = FakeTello('tello')
        self.drone = self.fake_tello

        self.declare_parameter('drone_type', 'fake_tello')
        self.handle_drone_type()

        self.drone_cmd_subscription = self.create_subscription(
            String,
            'drone_cmd',
            self.listener_drone_cmd_callback,
            10)

        self.drone_height_publisher = self.create_publisher(Float32, 'drone_height', 10)
        self.drone_battery_publisher = self.create_publisher(Float32, 'drone_battery', 10)
        self.drone_speed_publisher = self.create_publisher(Float32, 'drone_speed', 10)
        self.user_cmd_publisher = self.create_publisher(String, 'user_cmd', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_to_float_ros_msg(self, data):
        msg = Float32()
        msg.data = data
        return msg

    def create_none_user_cmd_ros_msg(self, user_cmd):
        msg = String()
        msg.data = user_cmd
        return msg

    def convert_drone_cmd_to_tello_cmd(self, drone_cmd):
        tello_cmd = TELLO_CMDS[drone_cmd]

        if drone_cmd != CMDS['TAKE_OFF'] or drone_cmd != CMDS['LAND']:
            tello_cmd = tello_cmd + ' %d' % 1

        return tello_cmd

    def handle_drone_type(self):
        drone_type = self.get_parameter('drone_type').get_parameter_value().string_value

        if self.drone.name != drone_type:
            self.get_logger().info('drone type %s!' % drone_type)

            self.drone.stop()
            if drone_type == 'fake_tello':
                self.drone = self.fake_tello
            else:
                self.drone = self.tello

            self.drone.start()

    def timer_callback(self):
        self.handle_drone_type()

        drone_height = self.drone.get_height()
        drone_battery = self.drone.get_battery()
        drone_speed = self.drone.get_speed()

        self.drone_height_publisher.publish(self.create_to_float_ros_msg(drone_height))
        self.drone_battery_publisher.publish(self.create_to_float_ros_msg(drone_battery))
        self.drone_speed_publisher.publish(self.create_to_float_ros_msg(drone_speed))

    def listener_drone_cmd_callback(self, msg):
        tello_cmd = self.convert_drone_cmd_to_tello_cmd(msg.data)

        if tello_cmd != CMDS['NONE']:
            self.drone.send_command(tello_cmd)
            self.get_logger().info('Tello cmd: %s' % tello_cmd)
        else:
            self.user_cmd_publisher.publish(self.create_none_user_cmd_ros_msg(tello_cmd))


def main(args=None):
    rclpy.init(args=args)

    drone_node = DroneNode()

    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
