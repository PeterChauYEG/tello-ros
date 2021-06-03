import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray
import numpy as np

from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT
from tellobot.pose_ml import PoseML

class PoseMLNode(Node):
    def __init__(self):
        super().__init__('pose_node')

        self.pose_ml = PoseML()

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_callback,
            1)
        self.pose_publisher = self.create_publisher(String, 'pose', 1)
        self.pose_points_publisher = self.create_publisher(Int16MultiArray, 'pose_points', 1)

    def convert_ros_msg_to_frame(self, msg):
        current_frame = np.array(msg.data)

        if len(current_frame) != TARGET_FRAME_HEIGHT*TARGET_FRAME_WIDTH*3:
            return

        reshaped_frame = current_frame.reshape(TARGET_FRAME_HEIGHT, TARGET_FRAME_WIDTH, 3)
        return reshaped_frame

    def convert_pose_points_to_ros_msg(self, pose_points):
        flattened_pose_points = np.array(pose_points).reshape(-1).tolist()
        msg = Int16MultiArray()
        msg.data = flattened_pose_points
        return msg

    def convert_pose_to_ros_msg(self, pose):
        msg = String()
        msg.data = pose
        return msg

    def listener_callback(self, msg):
        resized_frame = self.convert_ros_msg_to_frame(msg)

        self.pose_ml.set_current_frame(resized_frame)
        pose, pose_points = self.pose_ml.read()

        if pose is not None:
            msg = self.convert_pose_to_ros_msg(pose)
            self.pose_publisher.publish(msg)

        if len(pose_points) != 0:
            msg = self.convert_pose_points_to_ros_msg(pose_points)
            self.pose_points_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    pose_ml_node = PoseMLNode()

    rclpy.spin(pose_ml_node)

    pose_ml_node.pose_ml.stop()
    pose_ml_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
