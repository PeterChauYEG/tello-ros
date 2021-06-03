import socket
from threading import Thread
import numpy as np
import h264decoder
import cv2

from tellobot.resolutions import TELLO_CAMERA_FRAME_WIDTH, TELLO_CAMERA_FRAME_HEIGHT, TARGET_FRAME_WIDTH, \
    TARGET_FRAME_HEIGHT


class TelloCamera:
    def __init__(self):
        self.name = 'tello_camera'

        self.thread_started = False
        self.frame = None
        self.grabbed = False
        self.socket_video = None
        self.packet_data = b""
        self.raw_frame = None
        self.decoder = h264decoder.H264Decoder()
        self.thread = Thread(target=self.update, args=(), daemon=True)

    def get_frame(self):
        try:
            res_string, ip = self.socket_video.recvfrom(2048)
            self.packet_data = b"".join([self.packet_data, res_string])

            if len(res_string) != 1460:
                for frame in self.h264_decode(self.packet_data):
                    self.raw_frame = frame
                    self.grabbed = False
                    self.packet_data = b""

        except socket.error as exc:
            print("Caught exception socket.error : %s" % exc)

    def read_frame(self):
        if self.grabbed is not None and self.raw_frame is not None:
            resized_frame = cv2.resize(self.raw_frame, dsize=(TARGET_FRAME_WIDTH, TARGET_FRAME_HEIGHT),
                                       interpolation=cv2.INTER_LINEAR)
            recolored_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
            self.frame = recolored_frame.reshape(-1).tolist()
            self.grabbed = True
            self.raw_frame = None

        return self.grabbed, self.frame

    def start(self):
        self.thread_started = True
        tello_ip_port = ('192.168.10.1', 8889)
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_video.bind(('', 11111))

        self.socket_video.sendto(b'command', tello_ip_port)
        self.socket_video.sendto(b'streamon', tello_ip_port)

        self.thread.start()

    def update(self):
        while True:
            if not self.thread_started:
                return

            self.get_frame()

    def h264_decode(self, packet_data):
        res_frame_list = []
        frames = self.decoder.decode(packet_data)

        for framedata in frames:
            (frame, w, h, ls) = framedata

            if frame is not None:
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = frame.reshape(TELLO_CAMERA_FRAME_HEIGHT, TELLO_CAMERA_FRAME_WIDTH, 3)
                res_frame_list.append(frame)

        return res_frame_list

    def stop(self):
        self.thread_started = False

    def __del__(self):
        self.thread.stop()

        if self.socket_video is not None:
            self.socket_video.close()
