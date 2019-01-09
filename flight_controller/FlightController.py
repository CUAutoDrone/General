import pigpio
import numpy as np
from flight_controller import *
import threading
import time


# represents the flight controller
class FlightController(object):

    # constructor for the flight control
    # initialize proportional gain, integral gain, derivative gain
    # boolean variable to determine if device is armed
    def __init__(self, kp_gain=np.array, ki_gain=np.array, kd_gain=np.array, the_receiver=Receiver,
                 the_imu=IMU, the_motor=Motor):
        self.Kp = kp_gain
        self.Ki = ki_gain
        self.Kd = kd_gain
        self.receiver = the_receiver
        self.imu = the_imu
        self.motor = the_motor
        self.armed = False
        self.pi = None

    # getter for armed status
    @property
    def armed(self):
        return self._armed

    # setter for armed status
    @armed.setter
    def armed(self, armed):
        self._armed = armed

    # getter for proportional gain
    @property
    def Kp(self):
        return self._Kp

    # setter for proportional gain
    @Kp.setter
    def Kp(self, Kp):
        self._Kp = Kp

    # getter for integral gain
    @property
    def Ki(self):
        return self._Ki

    # setter for integral gain
    @Ki.setter
    def Ki(self, Ki):
        self._Ki = Ki

    # getter for derivative gain
    @property
    def Kd(self):
        return self._Kd

    # setter for derivative gain
    @Kd.setter
    def Kd(self, Kd):
        self._Kd = Kd

    # getter for receiver
    @property
    def receiver(self):
        return self._receiver

    # setter for receiver
    @receiver.setter
    def receiver(self, receiver):
        self._receiver = receiver

    # getter for motor
    @property
    def motor(self):
        return self._motor

    # setter for motor
    @motor.setter
    def motor(self, motor):
        self._motor = motor

    # getter for imu
    @property
    def imu(self):
        return self._imu

    # setter for imu
    @imu.setter
    def imu(self, imu):
        self._imu = imu

    # TODO: pre-flight check list
    # goes through list of Pre-Flight Checks
    def pre_flight_checks(self):
        passed = False
        # ensures throttle is at zero
        if self.receiver.throttle_safe():
            passed = True

        return passed

    # TODO: kill switch. May want separate methods for immediate and safe landing
    # kill switch for the vehicle
    def kill_switch(self):
        self.receiver.ARM = False

    # Updates current PID
    def compute_PID(self):

        # begin the thread
        threading.Timer(self.imu.sample_time, self.compute_PID).start()

        # used to determine how long one iteration of the method takes
        start_time = self.pi.get_current_tick()

        # Maps control input into angles
        control_angles = self.receiver.map_control_input()

        # Calculate Euler angles
        self.imu.calculate_angles(self.pi)

        # Compute errors in pitch and roll and yaw rate
        # error = setpoint - input
        error = np.array([0 - self.imu.euler_state[0],
                          0 - self.imu.euler_state[1],
                          0])

        # I_term replaces the error sum since it allows for smoother live PID tunings
        self.imu.I_term += np.multiply(self.Ki, error)

        # compute d input
        # d_input = input - last input
        d_input = np.array([self.imu.euler_state[0]-self.imu.prev_d_input[0],
                            self.imu.euler_state[1]-self.imu.prev_d_input[1], 0])

        # PID Law
        output = np.multiply(self.Kp, error) + self.imu.I_term - \
                 np.multiply(self.Kd, self.imu.prev_d_input / self.imu.sample_time)

        # ensures that the output also falls within output limitations,
        # as well as clamps the I_term after the output has been computed
        output = self.imu.check_output_limitations(output[0], output[1], output[2])

        self.imu.prev_d_input = d_input

        # Map controls into vector
        ctrl = np.array([control_angles[3], output[0], output[1], output[2]])

        wm = Motor.map_motor_output(ctrl)
        print(wm)

        Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, wm[0])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR2, wm[1])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR3, wm[2])
        Motor.set_motor_pulse(self.pi, self.motor.MOTOR4, wm[3])

        # time of end of the method
        end_time = self.pi.get_current_tick()
        self.imu.actual_time_length_of_PID_loop = (start_time - end_time) / 1e6

    # run the flight controller
    def run(self):
        self.pi = pigpio.pi()
        print(self.pi.connected)

        # set receiver input pins
        self.pi.set_mode(self.receiver.RECEIVER_CH1, pigpio.INPUT)
        self.pi.set_mode(self.receiver.RECEIVER_CH2, pigpio.INPUT)
        self.pi.set_mode(self.receiver.RECEIVER_CH3, pigpio.INPUT)
        self.pi.set_mode(self.receiver.RECEIVER_CH4, pigpio.INPUT)
        self.pi.set_mode(self.receiver.RECEIVER_CH5, pigpio.INPUT)
        print("Receiver input pins set")

        # initialize callbacks
        cb1 = self.pi.callback(self.receiver.RECEIVER_CH1, pigpio.EITHER_EDGE, self.receiver.cbf1)
        cb2 = self.pi.callback(self.receiver.RECEIVER_CH2, pigpio.EITHER_EDGE, self.receiver.cbf2)
        cb3 = self.pi.callback(self.receiver.RECEIVER_CH3, pigpio.EITHER_EDGE, self.receiver.cbf3)
        cb4 = self.pi.callback(self.receiver.RECEIVER_CH4, pigpio.EITHER_EDGE, self.receiver.cbf4)
        cb5 = self.pi.callback(self.receiver.RECEIVER_CH5, pigpio.EITHER_EDGE, self.receiver.cbf5)
        print("Callbacks initialized")

        # set motor output pins
        self.pi.set_mode(self.motor.MOTOR1, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.MOTOR2, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.MOTOR3, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.MOTOR4, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.MOTOR4, pigpio.OUTPUT)
        print("Motor output pins set")

        # set PWM frequencies
        self.pi.set_PWM_frequency(self.motor.MOTOR1, 400)
        self.pi.set_PWM_frequency(self.motor.MOTOR2, 400)
        self.pi.set_PWM_frequency(self.motor.MOTOR3, 400)
        self.pi.set_PWM_frequency(self.motor.MOTOR4, 400)
        print("PWM frequency set")

        # setup IMU
        self.imu.setupMPU6050(self.pi)

        # determine acceleration and gyroscopic offsets
        self.imu.update_accelerometer_offsets(self.pi)
        self.imu.update_gyroscope_offsets(self.pi)

        # machine loop
        while True:
            # TODO: Necessary? Does the same thing as self.motor.arm(pi)
            # send zero signal to motors
            # Motor.set_motor_pulse(self.pi, self.motor.MOTOR1, 1)
            # Motor.motor.set_motor_pulse(self.pi, self.motor.MOTOR2, 1)
            # Motor.motor.set_motor_pulse(self.pi, self.motor.MOTOR3, 1)
            # Motor.motor.set_motor_pulse(self.pi, self.motor.MOTOR4, 1)

            while self.armed is False:
                if self.receiver.ARM is True:
                    # perform pre-flight checks
                    if self.pre_flight_checks():
                        self.motor.arm(self.pi)
                        self.armed = True

            # flight loop
            while self.receiver.ARM is True and self.armed is True:
                self.compute_PID()
