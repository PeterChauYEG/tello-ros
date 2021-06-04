class FakeDrone:
  def __init__(self):
    self.name = 'fake_drone'
    self.started = False

  def start(self):
    if not self.started:
      self.started = True

  def stop(self):
    if self.started:
      self.started = False

  def send_command(self, command):
    print('send command: %s' % command)

  def get_response(self):
    return 'ok'

  def get_battery(self):
    return 100

  def get_speed(self):
    return 1

  def land(self):
    return True

  def takeoff(self):
    return True

  def flip(self, direction):
    return direction
