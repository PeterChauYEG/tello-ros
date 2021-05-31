import cv2
from threading import Thread

VIDEO_CAPTURE_DEVICE = 1
# WINDOW_WIDTH = 640
# WINDOW_HEIGHT = 480
WINDOW_WIDTH = 960
WINDOW_HEIGHT = 720
WINDOW_BRIGHTNESS = 150


class WebCamera:
    def __init__(self):
        self.name = 'web_camera'

        self.thread_started = False
        self.feed = None
        self.grabbed = False
        self.frame = None

        self.thread = Thread(target=self.update, args=(), daemon=True)

    def get_frame(self):
        self.grabbed, self.frame = self.feed.read()

    def read_frame(self):
        return self.grabbed, self.frame

    def start(self):
        self.thread_started = True
        self.feed = cv2.VideoCapture(VIDEO_CAPTURE_DEVICE)
        self.feed.set(3, WINDOW_WIDTH)
        self.feed.set(4, WINDOW_HEIGHT)
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
        pass

