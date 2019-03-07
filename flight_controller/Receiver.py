import numpy as np


# a class representing the receiver
class Receiver(object):
    def __init__(self, CH1, CH2, CH3, CH4, CH5):
        self.RECEIVER_CH1 = CH1
        self.RECEIVER_CH2 = CH2
        self.RECEIVER_CH3 = CH3
        self.RECEIVER_CH4 = CH4
        self.RECEIVER_CH5 = CH5
        self.ARM = False

        # VARIABLES FOR PWM MEASUREMENT
        self.rising_1 = 0
        self.pulse_width_ch1 = 0
        self.rising_2 = 0
        self.pulse_width_ch2 = 0
        self.rising_3 = 0
        self.pulse_width_ch3 = 0
        self.rising_4 = 0
        self.pulse_width_ch4 = 0
        self.rising_5 = 0
        self.pulse_width_ch5 = 0

    def cbf1(self, gpio, level, tick):

        # If rising edge, store time
        if level == 1:
            self.rising_1 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - self.rising_1
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                self.pulse_width_ch1 = width / 1000

    def cbf2(self, gpio, level, tick):

        # If rising edge, store time
        if level == 1:
            self.rising_2 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - self.rising_2
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                self.pulse_width_ch2 = width / 1000

    def cbf3(self, gpio, level, tick):

        # If rising edge, store time
        if level == 1:
            self.rising_3 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - self.rising_3
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                self.pulse_width_ch3 = width / 1000

    def cbf4(self, gpio, level, tick):

        # If rising edge, store time
        if level == 1:
            self.rising_4 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - self.rising_4
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                self.pulse_width_ch4 = width / 1000

    def cbf5(self, gpio, level, tick):

        # If rising edge, store time
        if level == 1:
            self.rising_5 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - self.rising_5
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                self.pulse_width_ch5 = width / 1000
                if self.pulse_width_ch5 > 1.4:
                    self.ARM = True
                else:
                    self.ARM = False

    @staticmethod
    def map(num, a, b, c, d):
        y = (num - a) * (d - c) / (b - a) + c
        return y

    def map_control_input(self):

        roll = Receiver.map(self.pulse_width_ch2, 1, 2, -30, 30)
        pitch = Receiver.map(self.pulse_width_ch4, 1, 2, -30, 30)
        yaw = Receiver.map(self.pulse_width_ch1, 1, 2, -10, 10)
        throttle = self.pulse_width_ch3
        arm = self.pulse_width_ch5

        return np.array([roll, pitch, yaw, throttle, arm])

    def throttle_safe(self):
        if self.pulse_width_ch3 > 1.0:
            print("Throttle is not zero")
            return False
        else:
            print("Throttle is zeroed")
            return True
