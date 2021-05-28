# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import UInt8MultiArray # Image is the message type
import cv2 # OpenCV library
import numpy as np

class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'video_frames', 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        VIDEO_CAPTURE_DEVICE = 1
        WINDOW_WIDTH = 600
        WINDOW_HEIGHT = 500
        WINDOW_BRIGHTNESS = 150
        WINDOW = "ML SHIT"
        cv2.namedWindow(WINDOW)

        self.cap = cv2.VideoCapture(VIDEO_CAPTURE_DEVICE)
        self.cap.set(3, WINDOW_WIDTH)
        self.cap.set(4, WINDOW_HEIGHT)
        self.cap.set(10, WINDOW_BRIGHTNESS)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()
        #
        if ret == True:
            if frame is not None:
        #         cv2.imshow("camera", frame)
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
                new_frame =frame
                newer_frame = new_frame.reshape(-1).tolist()

                data = UInt8MultiArray()
                data.data = newer_frame
                # current_frame = np.array(data.data)
                # newest_frame = current_frame.reshape(480,640,3)
                #
                # # Display image
                self.publisher_.publish(data)
                # cv2.imshow("ML SHIT", newest_frame)

                cv2.waitKey(1)
        #
        # Display the message on the console
        self.get_logger().info('Publishing video frame')

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher()

    # ret, frame = image_publisher.cap.read()
    #
    # if ret == True:
    #     if frame is not None:
    #         cv2.imshow("camera", frame)
    #         cv2.waitKey(1)

# Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
