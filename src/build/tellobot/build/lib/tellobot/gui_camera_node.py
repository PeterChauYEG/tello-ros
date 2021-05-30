import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray, Float32, Bool
import numpy as np

from tellobot.gui_camera import GUICamera
from tellobot.cmds import CMDS
from tellobot.camera import WINDOW_WIDTH, WINDOW_HEIGHT

class GUICCameraNode(Node):
    def __init__(self):
        super().__init__('gui_camera_node')

        self.pose = '--'
        self.pose_points = []
        self.drone_cmd = CMDS['NONE']
        self.is_pose_in_box = False

        self.gui = GUICamera()

        self.video_frame_subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_video_frames_callback,
            1)

        self.pose_subscription = self.create_subscription(
            String,
            'pose',
            self.listener_pose_callback,
            10)

        self.is_pose_in_box_subscription = self.create_subscription(
            Bool,
            'is_pose_in_box',
            self.listener_is_pose_in_box_callback,
            10)

        self.pose_points_subscription = self.create_subscription(
            Int16MultiArray,
            'pose_points',
            self.listener_pose_points_callback,
            10)

    def convert_ros_msg_to_frame(self, msg):
        current_frame = np.array(msg.data)
        resized_frame = current_frame.reshape(WINDOW_HEIGHT, WINDOW_WIDTH, 3)
        return resized_frame

    def listener_pose_callback(self, msg):
        self.pose = msg.data

    def listener_is_pose_in_box_callback(self, msg):
        self.is_pose_in_box = msg.data

    def listener_pose_points_callback(self, msg):
        pose_points = np.array(msg.data)
        resized_frame = pose_points.reshape(15,2).tolist()
        self.pose_points = resized_frame

    def listener_video_frames_callback(self, msg):
        resized_frame = self.convert_ros_msg_to_frame(msg)

        self.gui.update_image(resized_frame, self.pose_points, self.pose, self.is_pose_in_box)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    gui_camera_node = GUICCameraNode()

    rclpy.spin(gui_camera_node)
    gui_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
