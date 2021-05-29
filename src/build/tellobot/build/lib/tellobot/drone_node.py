import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray, Float32, Bool

from tellobot.gui import GUI

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        self.gui = GUI()

        self.drone_cmd_subscription = self.create_subscription(
            String,
            'drone_cmd',
            self.listener_drone_cmd_callback,
            10)

        self.drone_height_publisher = self.create_publisher(Float32, 'drone_height', 10)
        self.drone_battery_publisher = self.create_publisher(Float32, 'drone_battery', 10)
        self.drone_speed_publisher = self.create_publisher(Float32, 'drone_speed', 10)

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_to_float_ros_msg(self, data):
        msg = Float32()
        msg.data = data
        return msg

    def convert_drone_cmd_to_tello_cmd(self, drone_cmd):
        return drone_cmd

    def timer_callback(self):
        self.drone_height_publisher.publish(self.create_to_float_ros_msg(0.0))
        self.drone_battery_publisher.publish(self.create_to_float_ros_msg(1.0))
        self.drone_speed_publisher.publish(self.create_to_float_ros_msg(0.0))

    def listener_drone_cmd_callback(self, msg):
        tello_cmd = self.convert_drone_cmd_to_tello_cmd(msg.data)
        self.get_logger().info('tello cmd: %s' % tello_cmd)

def main(args=None):
    rclpy.init(args=args)

    drone_node = DroneNode()

    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
