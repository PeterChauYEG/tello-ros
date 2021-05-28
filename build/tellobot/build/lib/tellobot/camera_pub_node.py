import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from camera import Camera

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.camera = Camera()

        self.publisher_ = self.create_publisher(UInt8MultiArray, 'video_frames', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def convert_frame_to_ros_msg(self, frame):
        flattened_frame = frame.reshape(-1).tolist()
        msg = UInt8MultiArray()
        msg.data = flattened_frame
        return msg

    def timer_callback(self):
        rval, frame = self.camera.get_frame()

        if rval == True and frame is not None:
            msg = self.convert_frame_to_ros_msg
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
