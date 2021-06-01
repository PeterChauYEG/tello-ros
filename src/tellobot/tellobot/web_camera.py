import cv2
from threading import Thread

from tellobot.resolutions import WEB_CAMERA_FRAME_WIDTH, WEB_CAMERA_FRAME_HEIGHT
VIDEO_CAPTURE_DEVICE = 1
WINDOW_BRIGHTNESS = 150


class WebCamera:
    def __init__(self):
        self.name = 'web_camera'

        self.thread_started = False
        self.feed = None
        self.grabbed = False
        self.frame = None
        self.raw_frame = None
        self.thread = Thread(target=self.update, args=(), daemon=True)

    def get_frame(self):
        self.grabbed, self.raw_frame = self.feed.read()

    def read_frame(self):
        if self.grabbed and self.raw_frame is not None:
            self.frame = self.raw_frame.reshape(-1).tolist()
            self.raw_frame = None

        return self.grabbed, self.frame

    def start(self):
        self.thread_started = True
        self.feed = cv2.VideoCapture(VIDEO_CAPTURE_DEVICE)
        self.feed.set(3, WEB_CAMERA_FRAME_WIDTH)
        self.feed.set(4, WEB_CAMERA_FRAME_HEIGHT)
        self.feed.set(10, WINDOW_BRIGHTNESS)
        self.feed.set(cv2.CAP_PROP_FPS, 10)

        self.thread.start()

    def update(self):
        while True:
            if not self.thread_started:
                return

            self.get_frame()

    def stop(self):
        self.thread_started = False

    def __del__(self):
        self.thread.stop()

