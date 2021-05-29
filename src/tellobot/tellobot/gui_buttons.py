import cv2
import numpy as np
import tellobot.cvui as cvui
from tellobot.cmds import CMDS

# +++++++++++++===============================
WINDOW = "BUTTONS"


class GUIButtons:
    def __init__(self):
        cvui.init(WINDOW)
        self.frame = np.zeros((200, 500, 3), np.uint8)

    def draw_buttons(self, publish_user_cmd):
        if cvui.button(self.frame, 50, 50, "Take off"):
            publish_user_cmd(CMDS['TAKE_OFF'])

        if cvui.button(self.frame, 50, 100, "Land"):
            publish_user_cmd(CMDS['LAND'])

        if cvui.button(self.frame, 50, 150, "Reset"):
            publish_user_cmd(CMDS['NONE'])

    def show(self, publish_user_cmd):
        while True:
            self.frame[:] = (49, 52, 49)
            self.draw_buttons(publish_user_cmd)
            cvui.imshow(WINDOW, self.frame)

            if cv2.waitKey(20) == 27:
                break
