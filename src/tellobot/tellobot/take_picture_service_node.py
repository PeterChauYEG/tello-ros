import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import numpy as np
import cv2
import os
import datetime, time

from tellobot_interfaces.srv import TakePicture
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT

base_path = '/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/images'

class TakePictureService(Node):

    def __init__(self):
        super().__init__('take_picture_service_node')
        self.frame = None

        self.can_take_picture_start_time = time.time()
        self.dir_name = self.make_dir()
        self.current_picture_count = 0

        self.video_frame_subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_video_frames_callback,
            1)
        self.srv = self.create_service(TakePicture, 'take_picture', self.take_picture_cb)

    def make_dir(self):
        dir_name = os.path.join(base_path, datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%s'))
        os.mkdir(dir_name)
        return dir_name

    def get_file_path(self):
        filename = '%d.jpg' % self.current_picture_count
        file_path = os.path.join(self.dir_name, filename)
        self.current_picture_count += 1

        return file_path

    def listener_video_frames_callback(self, msg):
        current_frame = np.array(msg.data)
        reshaped_frame = current_frame.reshape(TARGET_FRAME_HEIGHT, TARGET_FRAME_WIDTH, 3)
        self.frame = reshaped_frame

    def take_picture_cb(self, request, response):
        self.get_logger().info('Incoming request')
        end_time = time.time()
        response.result = True

        if end_time - self.can_take_picture_start_time > 10:
            self.take_picture()
            self.can_take_picture_start_time = time.time()

        return response

    def take_picture(self):
        cv2.imwrite(self.get_file_path(), self.frame)


def main(args=None):
    rclpy.init(args=args)

    take_picture_service = TakePictureService()

    rclpy.spin(take_picture_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
