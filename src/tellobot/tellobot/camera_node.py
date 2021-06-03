import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32
import time

from tellobot.web_camera import WebCamera
from tellobot.tello_camera import TelloCamera


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        timer_period = 0.0001
        self.frames = 0
        self.fps = 0.00
        self.start_time = 0
        self.end_time = 0
        self.camera = None

        self.declare_parameter('camera_type', 'web_camera')
        self.handle_camera_type()

        self.publisher_ = self.create_publisher(UInt8MultiArray, 'video_frames', 1)
        self.fps_publisher = self.create_publisher(Float32, 'fps', 500)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def handle_camera_type(self):
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        if self.camera:
            self.camera.stop()

        if camera_type == 'web_camera':
            self.camera = WebCamera()
        else:
            self.camera = TelloCamera()

        self.camera.start()

    def convert_frame_to_ros_msg(self, frame):
        flattened_frame = frame
        msg = UInt8MultiArray()
        msg.data = flattened_frame
        return msg

    def convert_fps_to_ros_msg(self, fps):
        msg = Float32()
        msg.data = fps
        return msg

    def set_start_time(self):
        self.start_time = time.time()

    def calculate_fps(self):
        self.end_time = time.time()
        seconds = self.end_time - self.start_time
        self.fps = 1 / seconds

    def timer_callback(self):
        if self.camera and self.camera.thread_started:
            self.set_start_time()
            grabbed, frame = self.camera.read_frame()

            if grabbed and frame is not None:
                msg = self.convert_frame_to_ros_msg(frame)
                self.publisher_.publish(msg)
                self.calculate_fps()

        self.fps_publisher.publish(self.convert_fps_to_ros_msg(self.fps))


def main(args=None):
    rclpy.init(args=args)
    cam_pub_node = CameraNode()

    rclpy.spin(cam_pub_node)

    cam_pub_node.camera.stop()
    del cam_pub_node.camera
    cam_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
