import datetime
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from tellobot_interfaces.srv import TakePicture
from tellobot.resolutions import TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT

base_path = '/Users/peterchau/robotics/ros-drone/src/tellobot/tellobot/images'


class TakePictureService(Node):

  def __init__(self):
    super().__init__('take_picture_service_node')

    self.frame = None
    self.current_picture_count = 0
    self.enable_take_picture = False
    self.picture_timeout = 10

    self.can_take_picture_start_time = time.time()
    self.dir_name = None

    self.declare_parameter('picture_timeout', '10')
    self.declare_parameter('enable_take_picture', 'False')

    self.handle_params()

    self.video_frame_subscription = self.create_subscription(
      UInt8MultiArray,
      'video_frames',
      self.listener_video_frames_callback,
      1)
    self.srv = self.create_service(TakePicture, 'take_picture', self.take_picture_cb)

  def handle_params(self):
    picture_timeout = self.get_parameter('picture_timeout').get_parameter_value().integer_value
    enable_take_picture = self.get_parameter('enable_take_picture').get_parameter_value().bool_value

    self.enable_take_picture = enable_take_picture
    self.picture_timeout = picture_timeout

    if self.enable_take_picture:
      self.make_dir()

  def make_dir(self):
    dir_name = os.path.join(base_path, datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.mkdir(dir_name)
    self.dir_name = dir_name

  def get_file_path(self):
    filename = '%d.jpg' % self.current_picture_count
    file_path = os.path.join(self.dir_name, filename)
    self.current_picture_count += 1

    return file_path

  def listener_video_frames_callback(self, msg):
    current_frame = np.array(msg.data)

    if len(current_frame) != TARGET_FRAME_HEIGHT * TARGET_FRAME_WIDTH * 3:
      return

    reshaped_frame = current_frame.reshape(TARGET_FRAME_HEIGHT, TARGET_FRAME_WIDTH, 3)
    self.frame = reshaped_frame

  def take_picture_cb(self, request, response):
    end_time = time.time()
    response.result = True

    if self.enable_take_picture and end_time - self.can_take_picture_start_time > self.picture_timeout:
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
