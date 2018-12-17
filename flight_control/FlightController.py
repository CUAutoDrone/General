import pigpio
import numpy as np
from flight_controller import *


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

    # run flight loop
    def run(self):
        pi = pigpio.pi()
        print(pi.connected)

        # set receiver input pins
        pi.set_mode(self.receiver.RECEIVER_CH1, pigpio.INPUT)
        pi.set_mode(self.receiver.RECEIVER_CH1, pigpio.INPUT)
        pi.set_mode(self.receiver.RECEIVER_CH1, pigpio.INPUT)
        pi.set_mode(self.receiver.RECEIVER_CH1, pigpio.INPUT)
        pi.set_mode(self.receiver.RECEIVER_CH5, pigpio.INPUT)

        # initialize callbacks
        cb1 = pi.callback(self.receiver.RECEIVER_CH1, pigpio.EITHER_EDGE, Receiver.cbf1)
        cb2 = pi.callback(self.receiver.RECEIVER_CH2, pigpio.EITHER_EDGE, Receiver.cbf2)
        cb3 = pi.callback(self.receiver.RECEIVER_CH3, pigpio.EITHER_EDGE, Receiver.cbf3)
        cb4 = pi.callback(self.receiver.RECEIVER_CH4, pigpio.EITHER_EDGE, Receiver.cbf4)
        cb5 = pi.callback(self.receiver.RECEIVER_CH5, pigpio.EITHER_EDGE, Receiver.cbf5)

        # set motor output pins
        pi.set_mode(self.motor.MOTOR1, pigpio.OUTPUT)
        pi.set_mode(self.motor.MOTOR2, pigpio.OUTPUT)
        pi.set_mode(self.motor.MOTOR3, pigpio.OUTPUT)
        pi.set_mode(self.motor.MOTOR4, pigpio.OUTPUT)
        pi.set_PWM_frequency(self.motor.MOTOR1, 400)
        pi.set_PWM_frequency(self.motor.MOTOR2, 400)
        pi.set_PWM_frequency(self.motor.MOTOR3, 400)
        pi.set_PWM_frequency(self.motor.MOTOR4, 400)

        # setup IMU
        self.imu.MPU6050_handle, self.imu.acc_offsets, self.imu.gyro_offsets = self.imu.setupMPU6050(pi)

        # machine loop
        while True:
            self.motor.set_motor_pulse(pi, self.motor.MOTOR1, 1)
            self.motor.set_motor_pulse(pi, self.motor.MOTOR2, 1)
            self.motor.set_motor_pulse(pi, self.motor.MOTOR3, 1)
            self.motor.set_motor_pulse(pi, self.motor.MOTOR4, 1)

            # TODO: ^ is this redundant? does the same thing as Motor.arm(pi)

            # wait for arm
            while self.armed is False:
                if self.receiver.ARM is True:
                    # perform pre-flight checks
                    canarm = self.receiver.can_arm()
                    if canarm:
                        self.motor.arm(pi)
                        self.armed(True)

            # Initialize PID Control
            is_first_loop = 0
            err_sum = 0
            sys_time = pi.get_current_tick()

            # flight loop

            while self.receiver.ARM == 1:
                # Get Delta Time
                sys_time_new = pi.get_current_tick()
                dt = (sys_time_new - sys_time) / 1e6
                # correct for rollover
                if dt < 0:
                    dt = 0
                sys_time = sys_time_new

                # Maps control input into angles
                control_angles = self.receiver.map_control_input()

                # Get accelerometer and gyroscope data and compute angles
                accel_data = IMU.get_acceleration_data(pi, self.imu.MPU6050_handle) - IMU.get_acc_offsets
                gyro_data = IMU.get_gyroscope_data(pi, self.imu.MPU6050_handle) - IMU.get_gyro_offsets
                self.imu.euler_state = IMU.calculate_angles(pi, accel_data, gyro_data, dt, self.imu.euler_state)

                # Compute errors in pitch and roll and yaw rate
                err = np.array([0 - self.imu.euler_state[0],
                                0 - self.imu.euler_state[1],
                                0])

                # compute error integral
                err_sum = err_sum + err

                # PID law
                if is_first_loop == 0:
                    u = np.multiply(self.Kp, err) + np.multiply(self.Ki, dt * err_sum)
                    is_first_loop = 1
                else:
                    u = np.multiply(self.Kp, err) + np.multiply(self.Ki, dt * err_sum) + np.multiply(self.Kd, (err - prev_err) / dt)
                prev_err = err

                # Map controls into vector
                ctrl = np.array([control_angles[3], u[0], u[1], u[2]])

                # Map control angles into output signal and set motors
                wm = Motor.map_motor_output(ctrl)
                print(wm)
                self.motor.set_motor_pulse(pi, self.motor.MOTOR1, wm[0])
                self.motor.set_motor_pulse(pi, self.motor.MOTOR2, wm[1])
                self.motor.set_motor_pulse(pi, self.motor.MOTOR3, wm[2])
                self.motor.set_motor_pulse(pi, self.motor.MOTOR4, wm[3])
