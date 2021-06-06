import numpy as np

import tellobot.cvui as cvui
from tellobot.cmds import CMDS
from tellobot.colors import BACKGROUND_COLOR, TEXT_COLOR
from tellobot.gui_constants import UI_WINDOW

button_height = 32
button_width = 64
ui_height = 320
ui_width = 720
padding = 16
column_width = (ui_width - (padding * 2)) / 2
row_height = button_height + padding


class GUIButtons:
  def __init__(self):
    cvui.init(UI_WINDOW)
    self.frame = np.zeros((ui_height, ui_width, 3), np.uint8)
    self.cam_fps = 0
    self.drone_battery = 100
    self.drone_speed = 1
    self.drone_cmd = CMDS['NONE']
    self.pose = 0

  # pylint: disable=too-many-branches, too-many-statements
  def draw_buttons(self, publish_user_cmd):
    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "TAKE OFF"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['TAKE_OFF'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "LAND"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['LAND'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      row_height + padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "USER_CONTROL"):
      # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['USER_CONTROL'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "TRACK"):
      # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['TRACK'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "SENTRY"):
      # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['SENTRY'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      (row_height * 2) + padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "X_DEC"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['X_DEC'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "X_INC"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['X_INC'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Y_DEC"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['Y_DEC'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Y_INC"):
    # pylint: enable=no-value-for-parameter
      publish_user_cmd(CMDS['Y_INC'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      (row_height * 3) + padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Z_DEC"):
      publish_user_cmd(CMDS['Z_DEC'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Z_INC"):
      publish_user_cmd(CMDS['Z_INC'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Z_CW"):
      publish_user_cmd(CMDS['Z_CW'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "Z_CCW"):
      publish_user_cmd(CMDS['Z_CCW'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      (row_height * 4) + padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_L"):
      publish_user_cmd(CMDS['FLIP_L'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_R"):
      publish_user_cmd(CMDS['FLIP_R'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_F"):
      publish_user_cmd(CMDS['FLIP_F'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_B"):
      publish_user_cmd(CMDS['FLIP_B'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      column_width + (padding * 2),
      (row_height * 5) + padding,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_FL"):
      publish_user_cmd(CMDS['FLIP_FL'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_FR"):
      publish_user_cmd(CMDS['FLIP_FR'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_BL"):
      publish_user_cmd(CMDS['FLIP_BL'])

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "FLIP_BR"):
      publish_user_cmd(CMDS['FLIP_BR'])

    cvui.endRow()

    cvui.beginRow(
      self.frame,
      padding,
      ui_height - row_height,
      column_width,
      row_height,
      padding)

    # pylint: disable=no-value-for-parameter
    if cvui.button(button_width, button_height, "RESET"):
      publish_user_cmd(CMDS['NONE'])

    cvui.endRow()

  def draw_info(self):
    cvui.beginColumn(
      self.frame,
      padding,
      padding,
      column_width, -1,
      padding)

    # pylint: disable=no-value-for-parameter
    cvui.text(
      'Detected pose: {0}'.format(self.pose),
      0.4,
      TEXT_COLOR)

    # pylint: disable=no-value-for-parameter
    cvui.text(
      'Drone CMD: {0}'.format(self.drone_cmd),
      0.4,
      TEXT_COLOR)

    # pylint: disable=no-value-for-parameter
    cvui.text(
      'Drone Speed: {0} cm/s'.format(self.drone_speed),
      0.4,
      TEXT_COLOR)

    # pylint: disable=no-value-for-parameter
    cvui.text(
      'Drone Battery: {0}%'.format(self.drone_battery),
      0.4,
      TEXT_COLOR)

    # pylint: disable=no-value-for-parameter
    cvui.text(
      'CAM FPS: {0}'.format(self.cam_fps),
      0.4,
      TEXT_COLOR)

    cvui.endColumn()

  def show(self, publish_user_cmd):
    self.frame[:] = BACKGROUND_COLOR
    self.draw_buttons(publish_user_cmd)
    self.draw_info()
    cvui.imshow(UI_WINDOW, self.frame)
