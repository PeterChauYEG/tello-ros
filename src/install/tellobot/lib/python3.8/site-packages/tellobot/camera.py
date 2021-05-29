import cv2
from threading import Thread

VIDEO_CAPTURE_DEVICE = 1
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
WINDOW_BRIGHTNESS = 150


class Camera:
    def __init__(self):
        self.stream_stopped = False

        self.feed = cv2.VideoCapture(VIDEO_CAPTURE_DEVICE)
        self.feed.set(3, WINDOW_WIDTH)
        self.feed.set(4, WINDOW_HEIGHT)
        self.feed.set(10, WINDOW_BRIGHTNESS)
        self.feed.set(cv2.CAP_PROP_FPS, 10)

        self.grabbed, self.frame = self.feed.read()

    def get_frame(self):
        self.grabbed, self.frame = self.feed.read()

    def read_frame(self):
        return self.grabbed, self.frame

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stream_stopped:
                return

            self.get_frame()

    def stop(self):
        self.stream_stopped = True
