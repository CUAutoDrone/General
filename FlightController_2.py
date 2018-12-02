import pigpio
import time
import numpy as np

#I googled again, I can put IMU, motor, and receiver into separate modules and package it. Let me know if you'd rather do that.
#This is suggested if Python modules get too long. Also, need to test still. Please also look at GUI_Drone.py . Needs Python 3 environment
#Testing is limited, but it will update PID.

# a class representing the IMU
class IMU(object):
    def __init__(self, MPU6050_ADDR, alpha):
        self.MPU6050_ADDR = MPU6050_ADDR
        self.alpha = alpha
        self.euler_state = np.array([0, 0])
        self.accel_data = np.array([0,0])
        self.gyro_data = np.array([0, 0])

    # getter for euler state
    def get_euler_state(self):
        return self.euler_state

    # setter for euler state
    def set_euler_state(self, data):
        self.euler_state = data
        return self.euler_state

    # getter for acceleration data
    def get_accel_data(self):
        return self.accel_data

    # setter for acceleration data
    def set_accel_data(self, data):
        self.accel_data = data
        return self.accel_data

    # getter for gyroscopic data
    def get_gyro_data(self):
        return self.gyro_data

    # setter for gyroscopic data
    def set_gyro_data(self, data):
        self.gyro_data = data
        return self.gyro_data

    def get_acc_offsets(self, pi, MPU6050_handle):
        sum_acc_x = 0
        sum_acc_y = 0
        sum_acc_z = 0

        iter_num = 100

        for i in range(0, iter_num):
            AcX = (pi.i2c_read_byte_data(MPU6050_handle, 0x3B) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x3C)
            AcY = (pi.i2c_read_byte_data(MPU6050_handle, 0x3D) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x3E)
            AcZ = (pi.i2c_read_byte_data(MPU6050_handle, 0x3F) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x40)
            if AcX > 32768:
                AcX = AcX - 65536
            if AcY > 32768:
                AcY = AcY - 65536
            if AcZ > 32768:
                AcZ = AcZ - 65536

            sum_acc_x += AcX
            sum_acc_y += AcY
            sum_acc_z += AcZ

        AcX_mean = sum_acc_x / iter_num
        AcY_mean = sum_acc_y / iter_num
        AcZ_mean = sum_acc_z / iter_num

        # Convert to G's
        AcX_mean = AcX_mean / 65535 * 4
        AcY_mean = AcY_mean / 65535 * 4
        AcZ_mean = AcZ_mean / 65535 * 4

        return np.array([AcX_mean, AcY_mean, AcZ_mean + 1])

    def get_acceleration_data(self, pi, MPU6050_handle):
        AcX = (pi.i2c_read_byte_data(MPU6050_handle, 0x3B) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x3C)
        AcY = (pi.i2c_read_byte_data(MPU6050_handle, 0x3D) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x3E)
        AcZ = (pi.i2c_read_byte_data(MPU6050_handle, 0x3F) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x40)
        if AcX > 32768:
            AcX = AcX - 65536
        if AcY > 32768:
            AcY = AcY - 65536
        if AcZ > 32768:
            AcZ = AcZ - 65536

        # Convert to G's
        AcX = AcX * 4 / 65536
        AcY = AcY * 4 / 65536
        AcZ = AcZ * 4 / 65536

        return np.array([AcX, AcY, AcZ])

    def get_gyro_offsets(self, pi, MPU6050_handle):
        sum_gy_x = 0
        sum_gy_y = 0
        sum_gy_z = 0

        iter_num = 100

        for i in range(0, iter_num):
            GyX = (pi.i2c_read_byte_data(MPU6050_handle, 0x43) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x44)
            GyY = (pi.i2c_read_byte_data(MPU6050_handle, 0x45) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x46)
            GyZ = (pi.i2c_read_byte_data(MPU6050_handle, 0x47) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x48)
            if GyX > 32768:
                GyX = GyX - 65536
            if GyY > 32768:
                GyY = GyY - 65536
            if GyZ > 32768:
                GyZ = GyZ - 65536

            sum_gy_x += GyX
            sum_gy_y += GyY
            sum_gy_z += GyZ

        GyX_mean = sum_gy_x / iter_num
        GyY_mean = sum_gy_y / iter_num
        GyZ_mean = sum_gy_z / iter_num

        GyX_mean = GyX_mean / 65.5
        GyY_mean = GyY_mean / 65.5
        GyZ_mean = GyZ_mean / 65.5

        return np.array([GyX_mean, GyY_mean, GyZ_mean])

    def get_gyroscope_data(self, pi, MPU6050_handle):
        GyX = (pi.i2c_read_byte_data(MPU6050_handle, 0x43) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x44)
        GyY = (pi.i2c_read_byte_data(MPU6050_handle, 0x45) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x46)
        GyZ = (pi.i2c_read_byte_data(MPU6050_handle, 0x47) << 8) + pi.i2c_read_byte_data(MPU6050_handle, 0x48)
        if GyX > 32768:
            GyX = GyX - 65536
        if GyY > 32768:
            GyY = GyY - 65536
        if GyZ > 32768:
            GyZ = GyZ - 65536

        GyX = GyX / 65.5
        GyY = GyY / 65.5
        GyZ = GyZ / 65.5

        return np.array([GyX, GyY, GyZ])

    def setupMPU6050(self, pi):
        # opens connection at I2C bus 1
        mpu6050_handle = pi.i2c_open(1, self.MPU6050_ADDR, 0)

        # Configure things as done in:
        # https://github.com/tockn/MPU6050_tockn/blob/master/src/MPU6050_tockn.cpp

        pi.i2c_write_byte_data(mpu6050_handle, 0x19, 0x00)
        pi.i2c_write_byte_data(mpu6050_handle, 0x1a, 0x00)
        pi.i2c_write_byte_data(mpu6050_handle, 0x1b, 0x08)
        pi.i2c_write_byte_data(mpu6050_handle, 0x1c, 0x00)

        # Wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
        pi.i2c_write_byte_data(mpu6050_handle, 0x6B, 0x02)

        pi.i2c_write_byte_data(mpu6050_handle, 0x38, 1)

        # Set G Scale
        # Acc_Config = pi.i2c_read_byte_data(mpu6050_handle,0x1C)
        # Acc_Config_4G = (Acc_Config | 1<<3) & (~1<<4)

        # Calculate Offsets
        acc_offsets = self.get_acc_offsets(pi, mpu6050_handle)
        gyro_offsets = self.get_gyro_offsets(pi, mpu6050_handle)

        return mpu6050_handle, acc_offsets, gyro_offsets

    def calculate_angles(self, pi, accel_data, gyro_data, sys_time, euler_state):
        # Estimate angle from accelerometer
        pitch_acc = np.arctan2(accel_data[0], np.sqrt(np.power(accel_data[1], 2) + np.power(accel_data[2], 2))) * -1
        roll_acc = np.arctan2(accel_data[1], np.sqrt(np.power(accel_data[0], 2) + np.power(accel_data[2], 2)))

        # Complementary Filter
        acc_angles = np.array([roll_acc * 180 / np.pi, pitch_acc * 180 / np.pi])
        gyro_pr = np.array([gyro_data[0], gyro_data[1]])

        dt = (pi.get_current_tick() - sys_time) / 1e6
        if dt < 0:
            dt = 0

        new_angles = self.alpha * (euler_state + dt * gyro_pr) + (1 - self.alpha) * acc_angles

        return new_angles

# a class representing the motors
class Motor(object):
    def __init__(self, MOTOR1, MOTOR2, MOTOR3, MOTOR4):
        self.MOTOR1 = MOTOR1
        self.MOTOR2 = MOTOR2
        self.MOTOR3 = MOTOR3
        self.MOTOR4 = MOTOR4

    def map_motor_output(self, ctrl):
        throttle = ctrl[0]
        roll = ctrl[1]
        pitch = ctrl[2]
        yawrate = ctrl[3]

        m1 = throttle - roll + pitch - yawrate
        m2 = throttle - roll - pitch + yawrate
        m3 = throttle + roll - pitch - yawrate
        m4 = throttle + roll + pitch + yawrate
        return Motor.minMax(np.array([m1, m2, m3, m4]), 1, 2)

    def minMax(self, vec, min, max):
        for i in range(0, np.size(vec)):
            if vec[i] < min:
                vec[i] = min
            elif vec[i] > max:
                vec[i] = max
        return vec

    def set_motor_pulse(self, pi, gpio, timems):
        pi.set_PWM_dutycycle(gpio, timems / 2.5 * 255)

    def arm(self, pi):
        print("Arming...")
        Motor.set_motor_pulse(pi, self.MOTOR1, 1)
        Motor.set_motor_pulse(pi, self.MOTOR2, 1)
        Motor.set_motor_pulse(pi, self.MOTOR3, 1)
        Motor.set_motor_pulse(pi, self.MOTOR4, 1)

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


# represents the flight controller
class FlightController(object):

    # constructor for the flight control
    # initialize proportional gain, integral gain, derivative gain
    # boolean variable to determine if device is armed
    def __init__(self, kp_gain, ki_gain, kd_gain, the_receiver=Receiver, the_imu=IMU, the_motor=Motor):
        self.Kp = kp_gain
        self.Ki = ki_gain
        self.Kd = kd_gain

        self.receiver = the_receiver
        self.imu = the_imu
        self.motor = the_motor
        self.armed = False

    # getter / setter methods

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

            # ^ is this redundant? does the same thing as Motor.arm(pi)

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


def main():
    # receiver created with receiver channels initialized
    receiver = Receiver(17, 27, 22, 18, 23)
    # IMU created with MPU_6050, alpha initialized
    imu = IMU(0x68, 0.98)
    # motors created with motors initialized
    motor = Motor(10, 9, 25, 8)

    fc = FlightController(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), receiver, imu, motor)
    fc.run()


if __name__ == "__main__":
    main()
