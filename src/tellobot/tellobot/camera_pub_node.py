import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32
import time

from tellobot.camera import Camera

class CamPubNode(Node):
    def __init__(self):
        super().__init__('cam_pub_node')

        self.camera = Camera()
        self.camera.start()

        self.publisher_ = self.create_publisher(UInt8MultiArray, 'video_frames', 100)
        self.fps_publisher = self.create_publisher(Float32, 'fps', 60)
        timer_period = 0.0001
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frames = 0
        self.fps = 0.00
        self.start_time = 0
        self.end_time = 0

    def convert_frame_to_ros_msg(self, frame):
        flattened_frame = frame.reshape(-1).tolist()
        msg = UInt8MultiArray()
        msg.data = flattened_frame
        return msg

    def convert_fps_to_ros_msg(self, fps):
        msg = Float32()
        msg.data = fps
        return msg

    def calculate_fps(self):
        self.frames += 1

        if self.start_time == 0:
            self.start_time = time.time()

        self.end_time = time.time()
        seconds = self.end_time - self.start_time

        if seconds > 0:
            self.fps = self.frames / seconds

    def timer_callback(self):
        if self.fps > 10:
            grabbed, frame = self.camera.read_frame()

            if grabbed == True and frame is not None:
                msg = self.convert_frame_to_ros_msg(frame)
                self.publisher_.publish(msg)

        self.calculate_fps()
        self.fps_publisher.publish(self.convert_fps_to_ros_msg(self.fps))
def main(args=None):
    rclpy.init(args=args)
    cam_pub_node = CamPubNode()
    rclpy.spin(cam_pub_node)
    cam_pub_node.camera.stop()
    cam_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
