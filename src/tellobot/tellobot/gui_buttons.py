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
        self.cam_fps = 0
        self.drone_height = 0
        self.drone_battery = 0
        self.drone_speed = 0
        self.drone_cmd = 0
        self.pose = 0

    def draw_buttons(self, publish_user_cmd):
        if cvui.button(self.frame, 16, 16, "Take off"):
            publish_user_cmd(CMDS['TAKE_OFF'])

        if cvui.button(self.frame, 16, 64, "Land"):
            publish_user_cmd(CMDS['LAND'])

        if cvui.button(self.frame, 16, 112, "Reset"):
            publish_user_cmd(CMDS['NONE'])

    def draw_info(self):
        cvui.text(
            self.frame,
            200,
            16,
            'Detected pose: {0}'.format(self.pose))

        cvui.text(
            self.frame,
            200,
            32,
            'Drone CMD: {0}'.format(self.drone_cmd))

        cvui.text(
            self.frame,
            200,
            48,
            'Drone Speed: {0}'.format(self.drone_speed))

        cvui.text(
            self.frame,
            200,
            64,
            'Drone Battery: {0}'.format(self.drone_battery))

        cvui.text(
            self.frame,
            200,
            80,
            'Drone Height: {0}'.format(self.drone_height))

        cvui.text(
            self.frame,
            200,
            96,
            'CAM FPS: {0}'.format(self.cam_fps))

    def show(self, publish_user_cmd):
        self.frame[:] = (49, 52, 49)
        self.draw_buttons(publish_user_cmd)
        self.draw_info()
        cvui.imshow(WINDOW, self.frame)
