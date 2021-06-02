import socket
from threading import Thread, Timer
from tellobot.cmds import TELLO_CMDS

class TelloDrone:
    def __init__(self):
        self.name = 'tello_drone'
        self.tello_ip_port = ('192.168.10.1', 8889)
        self.command_timeout = .3
        self.imperial = False

        self.thread_started = False
        self.socket = None
        self.last_height = 0.0
        self.response = None
        self.abort_flag = False

        self.thread = Thread(target=self.update, args=(), daemon=True)
        self.auto_takeoff_thread = Thread(target=self.auto_take_off)

    def auto_take_off(self):
        """
        Firstly,it will waiting for the response that will be sent by Tello if Tello

        finish the takeoff command.If computer doesn't receive the response,it may be

        because tello doesn't takeoff normally,or because the UDP pack of response is

        lost.So in order to confirm the reason,computer will send 'height?'command to

        get several real-time height datas and get a average value.If the height is in

        normal range,tello will execute the moveup command.Otherwise,tello will land.

        Finally,the sending-command thread will start.
        """
        response = None
        height_tmp = 0  # temp variable to content value of height
        height_val = 0  # average value of height
        cnt = 0  # effective number of height reading

        # waiting for the response from tello
        while response != 'ok':
            # if self.quit_waiting_flag is True:
            #     break
            response = self.get_response()

        # receive the correct response
        if response == 'ok':
            self.send_command(TELLO_CMDS['z_inc'] + ' 0.5')

        # calculate the height of drone
        else:
            for i in range(0, 50):
                height_tmp = self.get_height()

                try:
                    height_val = height_val + height_tmp
                    cnt = cnt + 1
                except:
                    height_val = height_val

            height_val = height_val / cnt

            # if the height value is in normal range
            if 9 <= height_val <= 11:
                self.send_command(TELLO_CMDS['z_inc'] + ' 0.5')
            else:
                self.land()

        # # start the sendingCmd thread
        # self.sending_command_thread.start()

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

    def get_height(self):
        """Returns height(dm) of tello.

        Returns:
            int: Height(dm) of tello.

        """
        height = self.send_command('height?')
        height = str(height)
        print('height %s' % height)
        height = filter(str.isdigit, height)

        try:
            self.last_height = float(height)
            return self.last_height
        except:
            return self.last_height


    def get_battery(self):
        """Returns percent battery life remaining.

        Returns:
            int: Percent battery life remaining.

        """

        battery = self.send_command('battery?')

        try:
            battery = float(battery)
            return battery
        except:
            return 0.0


    def get_speed(self):
        speed = self.send_command('speed?')

        try:
            speed = float(speed)

            if self.imperial is True:
                speed = round((speed / 44.704), 1)
            else:
                speed = round((speed / 27.7778), 1)
            return speed
        except:
            return 0.0

    def land(self):
        return self.send_command('land')

    def takeoff(self):
        self.send_command('takeoff')
        takeoff_response = self.get_response()

        if takeoff_response != 'error':
            self.auto_takeoff_thread.start()
        else:
            print("battery low,please repalce with a new one")

    def flip(self, direction):
        return self.send_command('flip %s' % direction)

    def stop(self):
        self.thread_started = False

    def __del__(self):
        if self.socket is not None:
            self.socket.close()
