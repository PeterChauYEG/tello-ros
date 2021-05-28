import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2 # OpenCV library
from std_msgs.msg import UInt8MultiArray # Image is the message type
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'video_frames',
            self.listener_callback,
            10)

        # Used to convert between ROS and OpenCV images
        # self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        # current_frame = self.br.imgmsg_to_cv2(data)
        current_frame = np.array(data.data)
        new_frame = current_frame.reshape(480,640,3)

        # Display image
        cv2.imshow("ML SHIT", new_frame)

        cv2.waitKey(1)

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()# Basics ROS 2 program to subscribe to real-time streaming
