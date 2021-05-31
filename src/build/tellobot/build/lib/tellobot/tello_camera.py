import socket
from threading import Thread
import numpy as np
import h264decoder
import cv2


class TelloCamera:
    def __init__(self):
        self.name = 'tello_camera'

        self.thread_started = False
        self.frame = None
        self.grabbed = False

        self.decoder = h264decoder.H264Decoder()

        self.socket = None
        self.socket_video = None
        self.thread = Thread(target=self.update, args=())

    def get_frame(self):
        return self.frame

    def read_frame(self):
        return self.grabbed, self.frame

    def start(self):
        self.thread_started = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', 8889))
        self.socket_video.bind(('', 11111))

        self.socket.sendto(b'command', ('192.168.10.1', 8889))
        self.socket.sendto(b'streamon', ('192.168.10.1', 8889))
        self.thread.start()
        # self.socket.close()

    def update(self):
        """
        Listens for video streaming (raw h264) from the Tello.

        Runs as a thread, sets self.frame to the most recent frame Tello captured.

        """
        packet_data = b""

        while True:
            if not self.thread_started:
                return

            try:
                res_string, ip = self.socket_video.recvfrom(2048)
                packet_data = b"".join([packet_data, res_string])
                # end of frame
                if len(res_string) != 1460:
                    for frame in self.h264_decode(packet_data):
                        self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        self.grabbed = True

                    packet_data = b""

            except socket.error as exc:
                print ("Caught exception socket.error : %s" % exc)

    def h264_decode(self, packet_data):
        """
        decode raw h264 format data from Tello

        :param packet_data: raw h264 data array

        :return: a list of decoded frame
        """
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, int(ls / 3), 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list


    def stop(self):
        self.thread_started = False

    def __del__(self):
        if self.socket_video is not None:
            self.socket_video.close()

        if self.socket is not None:
            self.socket.close()
