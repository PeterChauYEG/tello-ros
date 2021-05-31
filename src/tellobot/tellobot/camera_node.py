import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32
import time

from tellobot.web_camera import WebCamera
from tellobot.tello_camera import TelloCamera

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.web_camera = WebCamera()
        self.tello_camera = TelloCamera()
        self.camera = self.web_camera

        self.declare_parameter('camera_type', 'web_camera')
        self.handle_camera_type()

        self.publisher_ = self.create_publisher(UInt8MultiArray, 'video_frames', 1)
        self.fps_publisher = self.create_publisher(Float32, 'fps', 60)
        timer_period = 0.0001
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frames = 0
        self.fps = 0.00
        self.start_time = 0
        self.end_time = 0

    def handle_camera_type(self):
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        self.camera.stop()

        if camera_type == 'web_camera':
            self.camera = self.web_camera
        else:
            self.camera = self.tello_camera
        
        print('tick %s' % self.camera.name)
        self.camera.start()

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
        if self.fps > 60:
            if self.camera.stream_stopped == False:
                grabbed, frame = self.camera.read_frame()

                if grabbed == True and frame is not None:
                    msg = self.convert_frame_to_ros_msg(frame)
                    self.publisher_.publish(msg)

        self.calculate_fps()
        self.fps_publisher.publish(self.convert_fps_to_ros_msg(self.fps))

def main(args=None):
    rclpy.init(args=args)
    cam_pub_node = CameraNode()
    rclpy.spin(cam_pub_node)
    cam_pub_node.camera.stop()
    cam_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
