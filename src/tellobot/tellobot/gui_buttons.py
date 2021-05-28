import cv2
import numpy as np
from tellobot.gui import NORMAL_COLOR, ACTIVE_COLOR
import tellobot.cvui as cvui

# +++++++++++++===============================
WINDOW = "BUTTONS"

class GUIButtons:
    def __init__(self):
        cvui.init(WINDOW)
        self.frame = np.zeros((200, 500, 3), np.uint8)
        # cv2.moveWindow(WINDOW, 800, 360)  # do this dynamically
        self.cmd = ''

    def draw_buttons(self, publish_user_cmd):
        if (cvui.button(self.frame, 110, 80, "Take off")):
            self.cmd = 'Take off'
            publish_user_cmd('Take off')

    def show(self, publish_user_cmd):
        while True:
            self.frame[:] = (49, 52, 49)
            self.draw_buttons(publish_user_cmd)

            cvui.printf(self.frame, 250, 90, 0.4, 0xff0000, "User CMD: %s", self.cmd)

            cvui.imshow(WINDOW, self.frame)

            if cv2.waitKey(20) == 27:
                break

