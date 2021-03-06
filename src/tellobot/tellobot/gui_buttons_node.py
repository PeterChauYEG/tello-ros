import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32
from tellobot.gui_buttons import GUIButtons
import cv2

class GUIButtonsNode(Node):
    def __init__(self):
        super().__init__('gui_buttons_node')

        self.user_cmd_publisher = self.create_publisher(String, 'user_cmd', 1)

        self.drone_battery_subscription = self.create_subscription(
            Int8,
            'drone_battery',
            self.drone_battery_callback,
            500)

        self.drone_speed_subscription = self.create_subscription(
            Int8,
            'drone_speed',
            self.drone_speed_callback,
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

        self.gui_buttons = GUIButtons()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def convert_user_cmd_to_ros_msg(self, user_cmd):
        msg = String()
        msg.data = user_cmd
        return msg

    def publish_user_cmd(self, user_cmd):
        self.user_cmd_publisher.publish(self.convert_user_cmd_to_ros_msg(user_cmd))

    def fps_callback(self, msg):
        self.gui_buttons.cam_fps = round(msg.data,2)

    def drone_battery_callback(self, msg):
        self.gui_buttons.drone_battery = msg.data

    def drone_speed_callback(self, msg):
        self.gui_buttons.drone_speed = msg.data

    def listener_drone_cmd_callback(self, msg):
        self.gui_buttons.drone_cmd = msg.data

    def listener_pose_callback(self, msg):
        self.gui_buttons.pose = msg.data

    def timer_callback(self):
        self.gui_buttons.show(self.publish_user_cmd)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    gui_buttons_node = GUIButtonsNode()

    rclpy.spin(gui_buttons_node)
    gui_buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
