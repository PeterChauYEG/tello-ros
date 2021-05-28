import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import UInt8MultiArray
import numpy as np
from gui import GUI

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')

        self.gui = GUI()

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_callback,
            10)

    def convert_ros_msg_to_frame(self, msg):
        current_frame = np.array(msg.data)
        resized_frame = current_frame.reshape(480,640,3)
        return resized_frame

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame -')
        resized_frame = self.convert_ros_msg_to_frame(msg)
        self.gui.update_image(resized_frame, False, [], {})

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
