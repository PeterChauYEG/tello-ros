class FakeTello:
    def __init__(self, name):
        self.name = name
        self.started = False

    def start(self):
        if self.started == False:
            self.started = True

    def stop(self):
        if self.started == True:
            self.started = False

    def send_command(self, command):
        print('send command: %s' % command)

    def get_response(self):
        return 'ok'

    def get_height(self):
        return 10.0

    def get_battery(self):
        return 10.0

    def get_speed(self):
        return 10.0

    def land(self):
        return True

    def takeoff(self):
        return True

    def flip(self, direction):
        return True
