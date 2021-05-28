import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Int16MultiArray, String, UInt8MultiArray, Float32
import numpy as np
import time

from tellobot.gui import GUI


class GUINode(Node):
    def __init__(self):
        super().__init__('video_subscriber')

        self.pose = None
        self.pose_points = []
        self.cam_fps = None
        self.gui_fps = None
        self.frames = 0
        self.start_time = 0
        self.end_time = 0

        self.gui = GUI()

        self.video_frame_subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_video_frames_callback,
            60)

        self.fps_subscription = self.create_subscription(
            Float32,
            'fps',
            self.fps_pose_callback,
            10)

        self.pose_subscription = self.create_subscription(
            String,
            'pose',
            self.listener_pose_callback,
            10)

        self.poe_points_subscription = self.create_subscription(
            Int16MultiArray,
            'pose_points',
            self.listener_poses_points_callback,
            10)

    def convert_ros_msg_to_frame(self, msg):
        current_frame = np.array(msg.data)
        resized_frame = current_frame.reshape(480, 640, 3)
        return resized_frame

    def calculate_fps(self):
        self.frames += 1

        if self.start_time == 0:
            self.start_time = time.time()

        self.end_time = time.time()
        seconds = self.end_time - self.start_time

        if seconds > 0:
            self.gui_fps  = round(self.frames / seconds, 2)

    def fps_pose_callback(self, msg):
        round_fps = round(msg.data, 2)
        self.cam_fps = round_fps
        self.calculate_fps()

    def listener_pose_callback(self, msg):
        self.pose = msg.data

    def listener_poses_points_callback(self, msg):
        pose_points = np.array(msg.data)
        resized_frame = pose_points.reshape(15,2).tolist()
        self.pose_points = resized_frame

    def listener_video_frames_callback(self, msg):
        resized_frame = self.convert_ros_msg_to_frame(msg)

        if self.gui.window_width is None or self.gui.center_box_points is None:
            self.gui.set_window_size(resized_frame)
            self.gui.get_center_box_points(resized_frame)

        self.gui.update_image(resized_frame, self.pose_points, {}, self.pose, '', self.cam_fps, self.gui_fps)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
