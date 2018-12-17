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

    # GLOBAL VARIABLES FOR PWM MEASUREMENT
    rising_1 = 0
    pulse_width_ch1 = 0
    rising_2 = 0
    pulse_width_ch2 = 0
    rising_3 = 0
    pulse_width_ch3 = 0
    rising_4 = 0
    pulse_width_ch4 = 0
    rising_5 = 0
    pulse_width_ch5 = 0

    def cbf1(self, gpio, level, tick):
        # connects global and local variables
        global rising_1
        global pulse_width_ch1

        # If rising edge, store time
        if level == 1:
            rising_1 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - rising_1
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                pulse_width_ch1 = width / 1000

    def cbf2(self, gpio, level, tick):
        # connects global and local variables
        global rising_2
        global pulse_width_ch2

        # If rising edge, store time
        if level == 1:
            rising_2 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - rising_2
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                pulse_width_ch2 = width / 1000

    def cbf3(self, gpio, level, tick):
        # connects global and local variables
        global rising_3
        global pulse_width_ch3

        # If rising edge, store time
        if level == 1:
            rising_3 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - rising_3
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                pulse_width_ch3 = width / 1000

    def cbf4(self, gpio, level, tick):
        # connects global and local variables
        global rising_4
        global pulse_width_ch4

        # If rising edge, store time
        if level == 1:
            rising_4 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - rising_4
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                pulse_width_ch4 = width / 1000

    def cbf5(self, gpio, level, tick):
        # connects global and local variables
        global rising_5
        global pulse_width_ch5
        global ARM

        # If rising edge, store time
        if level == 1:
            rising_5 = tick
        # If falling edge, subtract out rising time to get pulse width
        elif level == 0:

            width = tick - rising_5
            # in case of wraparound
            if width > 0:
                # divide by 1000 to get milliseconds
                pulse_width_ch5 = width / 1000
                if pulse_width_ch5 > 1.4:
                    ARM = 1
                else:
                    ARM = 0

    def map(self, num, a, b, c, d):
        y = (num - a) * (d - c) / (b - a) + c
        return y

    def map_control_input(self):
        global pulse_width_ch1
        global pulse_width_ch2
        global pulse_width_ch3
        global pulse_width_ch4
        global pulse_width_ch5

        roll = map(pulse_width_ch2, 1, 2, -30, 30)
        pitch = map(pulse_width_ch4, 1, 2, -30, 30)
        yaw = map(pulse_width_ch1, 1, 2, -10, 10)
        throttle = pulse_width_ch3
        arm = pulse_width_ch5

        return np.array([roll, pitch, yaw, throttle, arm])

    def can_arm(self):
        canarm = True
        if pulse_width_ch3 > 1.0:
            canarm = False
            print("Failed Preflight Check.  Throttle Not Zero")

        return canarm

