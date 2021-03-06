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
            self.response, ip = self.socket.recvfrom(3000)

        except socket.error as exc:
            print ("Caught exception socket.error : %s" % exc)

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

    def send_command(self, command):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :return (str): Response from Tello.

        """

        print (">> sending cmd: {}".format(command))
        self.abort_flag = False
        timer = Timer(self.command_timeout, self.set_abort_flag)

        self.socket.sendto(command.encode('utf-8'), self.tello_ip_port)

        timer.start()
        while self.response is None:
            if self.abort_flag is True:
                break
        timer.cancel()

        if self.response is None:
            response = 'none_response'
        else:
            response = self.response.decode('utf-8')

        self.response = None

        return response

    def set_abort_flag(self):
        """
        Sets self.abort_flag to True.

        Used by the timer in Tello.send_command() to indicate to that a response

        timeout has occurred.

        """

        self.abort_flag = True


    def get_response(self):
        response = self.response
        return response

    def get_battery(self):
        battery = self.send_command('battery?')

        return battery

    def get_speed(self):
        speed = self.send_command('speed?')

        return speed

    def land(self):
        return self.send_command('land')

    def takeoff(self):
        self.send_command('takeoff')

    def flip(self, direction):
        return self.send_command('flip %s' % direction)

    def stop(self):
        self.thread_started = False

    def __del__(self):
        if self.socket is not None:
            self.socket.close()
