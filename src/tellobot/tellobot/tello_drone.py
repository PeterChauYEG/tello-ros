import socket
from threading import Thread, Timer

class TelloDrone:
    def __init__(self):
        self.name = 'tello_drone'
        self.tello_ip_port = ('192.168.10.1', 8889)
        self.command_timeout = .3
        self.imperial = False

        self.thread_started = False
        self.socket = None
        self.response = None
        self.abort_flag = False
        self.thread = Thread(target=self.update, args=(), daemon=True)

    def get_data(self):
        try:
            response, ip = self.socket.recvfrom(3000)
            self.set_response(response)

        except socket.error as exc:
            print ("Caught exception socket.error : %s" % exc)

    def set_response(self, response):
        if response is None:
            self.response = None
        else:
            self.response = response.decode('utf-8')

    def start(self):
        self.thread_started = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', 8889))

        self.socket.sendto(b'command', self.tello_ip_port)

        self.thread.start()

    def update(self):
        while True:
            if not self.thread_started:
                return

            self.get_data()

    def set_abort_flag(self):
        """
        Sets self.abort_flag to True.

        Used by the timer in Tello.send_command() to indicate to that a response

        timeout has occurred.

        """

        self.abort_flag = True

    def send_command(self, command):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :return (str): Response from Tello.

        """
        self.set_response(None)

        print (">> sending cmd: {}".format(command))
        timer = Timer(self.command_timeout, self.set_abort_flag)
        self.socket.sendto(command.encode('utf-8'), self.tello_ip_port)

        timer.start()
        while self.response is None:
            if self.abort_flag is True:
                break
        timer.cancel()

        return self.response

    def get_battery(self):
        battery = self.send_command('battery?')

        try:
            return round(int(battery))
        except:
            return 0

    def get_speed(self):
        speed = self.send_command('speed?')

        try:
            return round(int(speed))
        except:
            return 0

    def land(self):
        return self.send_command('land')

    def takeoff(self):
        self.send_command('takeoff')

    def flip(self, direction):
        return self.send_command('flip %s' % direction)

    def stop(self):
        self.thread_started = False

    def __del__(self):
        if self.thread:
            self.thread.stop()
            
        if self.socket is not None:
            self.socket.close()
