import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tellobot.gui_buttons import GUIButtons

class GUIButtonsNode(Node):
    def __init__(self):
        super().__init__('gui_buttons_node')

        self.user_cmd_publisher = self.create_publisher(String, 'user_cmd', 10)

        self.gui_buttons = GUIButtons()
        self.gui_buttons.show(self.publish_user_cmd)

    def convert_user_cmd_to_ros_msg(self, user_cmd):
        msg = String()
        msg.data = user_cmd
        return msg

    def publish_user_cmd(self, user_cmd):
        self.user_cmd_publisher.publish(self.convert_user_cmd_to_ros_msg(user_cmd))

def main(args=None):
    rclpy.init(args=args)

    gui_buttons_node = GUIButtonsNode()

    rclpy.spin(gui_buttons_node)
    gui_buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
