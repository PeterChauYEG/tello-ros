import cv2
import numpy as np
import tellobot.cvui as cvui
from tellobot.cmds import CMDS

# +++++++++++++===============================
WINDOW = "BUTTONS"


class GUIButtons:
    def __init__(self):
        cvui.init(WINDOW)
        self.frame = np.zeros((200, 700, 3), np.uint8)
        self.cam_fps = 0
        self.drone_battery = 100
        self.drone_speed = 1
        self.drone_cmd = 0
        self.pose = 0

    def draw_buttons(self, publish_user_cmd):
        if cvui.button(self.frame, 16, 16, "Take off"):
            publish_user_cmd(CMDS['TAKE_OFF'])

        if cvui.button(self.frame, 16, 64, "Land"):
            publish_user_cmd(CMDS['LAND'])

        if cvui.button(self.frame, 16, 112, "Reset"):
            publish_user_cmd(CMDS['NONE'])

        if cvui.button(self.frame, 128, 16, "X_DEC"):
            publish_user_cmd(CMDS['X_DEC'])

        if cvui.button(self.frame, 128, 64, "X_INC"):
            publish_user_cmd(CMDS['X_INC'])

        if cvui.button(self.frame, 224, 16, "Y_DEC"):
            publish_user_cmd(CMDS['Y_DEC'])

        if cvui.button(self.frame, 224, 64, "Y_INC"):
            publish_user_cmd(CMDS['Y_INC'])

        if cvui.button(self.frame, 320, 16, "Z_DEC"):
            publish_user_cmd(CMDS['Z_DEC'])

        if cvui.button(self.frame, 320, 64, "Z_INC"):
            publish_user_cmd(CMDS['Z_INC'])

    def draw_info(self):
        cvui.text(
            self.frame,
            416,
            16,
            'Detected pose: {0}'.format(self.pose))

        cvui.text(
            self.frame,
            416,
            32,
            'Drone CMD: {0}'.format(self.drone_cmd))

        cvui.text(
            self.frame,
            416,
            48,
            'Drone Speed: {0} cm/s'.format(self.drone_speed))

        cvui.text(
            self.frame,
            416,
            64,
            'Drone Battery: {0}%'.format(self.drone_battery))

        cvui.text(
            self.frame,
            416,
            80,
            'CAM FPS: {0}'.format(self.cam_fps))


    def show(self, publish_user_cmd):
        self.frame[:] = (49, 52, 49)
        self.draw_buttons(publish_user_cmd)
        self.draw_info()
        cvui.imshow(WINDOW, self.frame)
