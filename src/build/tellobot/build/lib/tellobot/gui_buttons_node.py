import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray, Float32, Bool

from tellobot.gui_buttons import GUIButtons

class GUIButtonsNode(Node):
    def __init__(self):
        super().__init__('gui_buttons_node')

        self.drone_cmd = '--'
        self.gui_buttons = GUIButtons()
        self.gui_buttons.show(self.publish_user_cmd)

        self.drone_cmd_subscription = self.create_subscription(
            String,
            'drone_cmd',
            self.listener_drone_cmd_callback,
            10)

    def publish_user_cmd(self, user_cmd):
        print('user_cmd: %s' % user_cmd)


    def listener_drone_cmd_callback(self, msg):
        self.drone_cmd = msg.data

def main(args=None):
    rclpy.init(args=args)

    gui_buttons_node = GUIButtonsNode()

    rclpy.spin(gui_buttons_node)
    gui_buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
