import cv2

VIDEO_CAPTURE_DEVICE = 1
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
WINDOW_BRIGHTNESS = 150


class Camera:
    def __init__(self):
        self.feed = cv2.VideoCapture(VIDEO_CAPTURE_DEVICE)
        self.feed.set(3, WINDOW_WIDTH)
        self.feed.set(4, WINDOW_HEIGHT)
        self.feed.set(10, WINDOW_BRIGHTNESS)
        self.feed.set(cv2.CAP_PROP_FPS, 10)

    def get_frame(self):
        if self.feed.isOpened():
            rval, frame = self.feed.read()

            return rval,frame
