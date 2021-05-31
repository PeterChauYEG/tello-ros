import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, Bool
import numpy as np

from tellobot.ai import AI
from tellobot.cmds import CMDS

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

        self.user_cmd_subscription = self.create_subscription(
            String,
            'user_cmd',
            self.listener_user_cmd_callback,
            10)

    def convert_drone_cmd_to_ros_msg(self, drone_cmd):
        msg = String()
        msg.data = drone_cmd
        return msg

    def convert_is_pose_in_box_to_ros_msg(self, is_pose_in_box):
        msg = Bool()
        msg.data = is_pose_in_box
        return msg

    def listener_user_cmd_callback(self, msg):
        self.ai.update_user_cmd(msg.data)
        self.drone_cmd_publisher.publish(self.convert_drone_cmd_to_ros_msg(self.ai.drone_cmd))

    def listener_pose_callback(self, msg):
        self.ai.update_current_pose(msg.data)

    def listener_pose_points_callback(self, msg):
        pose_points = np.array(msg.data)
        resized_pose_points = pose_points.reshape(15,2).tolist()
        self.ai.get_sum_of_distance(resized_pose_points)
        self.ai.get_is_pose_in_box()

        if self.ai.user_cmd == CMDS['NONE']:
            self.ai.get_center_human_cmd()

        self.drone_cmd_publisher.publish(self.convert_drone_cmd_to_ros_msg(self.ai.drone_cmd))
        self.is_pose_in_box_publisher.publish(self.convert_is_pose_in_box_to_ros_msg(self.ai.is_pose_in_box))

        if self.ai.user_cmd == '':
            self.ai.reset_state()


def main(args=None):
    rclpy.init(args=args)

    ai_node = AINode()

    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
