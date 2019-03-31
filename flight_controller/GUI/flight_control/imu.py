import numpy as np


# a class representing the IMU
class IMU(object):
    def __init__(self, the_MPU6050_ADDR, the_alpha):
        self.MPU6050_ADDR = the_MPU6050_ADDR
        self.alpha = the_alpha
        self.euler_state = np.array([0.0, 0.0])
        self.accel_data = np.array([0.0, 0.0, 0.0])
        self.gyro_data = np.array([0.0, 0.0, 0.0])
        self.acc_offsets = None
        self.gyro_offsets = None
        self.mpu6050_handle = None

        # I term
        self.I_term = np.array([0.0,0.0,0.0])

        # TODO: request actual maximum output of PWM measurements
        # the maximum output
        self.output_max = 255.0

        # the minimum input
        self.output_min = 0.0

        # time in between PID update intervals
        self.sample_time = 1.0
        # time it takes to complete one PID loop
        self.actual_time_length_of_PID_loop = 0.0

        # the previous derivative input
        self.prev_d_input = np.array([0.0, 0.0, 0.0])

    # getter for euler state
    @property
    def euler_state(self):
        return self._euler_state

    # setter for euler state
    @euler_state.setter
    def euler_state(self, euler_state):
        self._euler_state = euler_state

    # getter for acceleration data
    @property
    def accel_data(self):
        return self._accel_data

    # setter for acceleration data
    @accel_data.setter
    def accel_data(self, accel_data):
        self._accel_data = accel_data

    # getter for gyroscopic data
    @property
    def gyro_data(self):
        return self._gyro_data

    # setter for gyroscopic data
    @gyro_data.setter
    def gyro_data(self, gyro_data):
        self._gyro_data = gyro_data

    # getter for mpu6050_handle data
    @property
    def mpu6050_handle(self):
        return self._mpu6050_handle

    # setter for mpu6050_handle data
    @mpu6050_handle.setter
    def mpu6050_handle(self, mpu6050_handle):
        self._mpu6050_handle = mpu6050_handle

    # getter for alpha
    @property
    def alpha(self):
        return self._alpha

    # setter for alpha
    @alpha.setter
    def alpha(self, alpha):
        self._alpha = alpha

    def update_accelerometer_offsets(self, pi):
        sum_acc_x = 0
        sum_acc_y = 0
        sum_acc_z = 0

        iter_num = 100

        for i in range(0, iter_num):
            AcX = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3B) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x3C)
            AcY = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3D) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x3E)
            AcZ = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3F) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x40)
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

        self.acc_offsets = np.array([AcX_mean, AcY_mean, AcZ_mean + 1])
        print("Accelerometer offsets updated")
        print()

    def update_accelerometer_data(self, pi):
        AcX = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3B) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x3C)
        AcY = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3D) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x3E)
        AcZ = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x3F) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x40)
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

    def update_gyroscope_offsets(self, pi):
        sum_gy_x = 0
        sum_gy_y = 0
        sum_gy_z = 0

        iter_num = 100

        for i in range(0, iter_num):
            GyX = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x43) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x44)
            GyY = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x45) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x46)
            GyZ = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x47) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x48)
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

        self.gyro_offsets = np.array([GyX_mean, GyY_mean, GyZ_mean])
        print("Gyroscope offsets updated")
        print()

    def update_gyroscope_data(self, pi):
        GyX = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x43) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x44)
        GyY = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x45) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x46)
        GyZ = (pi.i2c_read_byte_data(self.mpu6050_handle, 0x47) << 8) + pi.i2c_read_byte_data(self.mpu6050_handle, 0x48)
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
        mpu6050_handler = pi.i2c_open(1, self.MPU6050_ADDR, 0)

        # Configure things as done in:
        # https://github.com/tockn/MPU6050_tockn/blob/master/src/MPU6050_tockn.cpp

        pi.i2c_write_byte_data(mpu6050_handler, 0x19, 0x00)
        pi.i2c_write_byte_data(mpu6050_handler, 0x1a, 0x00)
        pi.i2c_write_byte_data(mpu6050_handler, 0x1b, 0x08)
        pi.i2c_write_byte_data(mpu6050_handler, 0x1c, 0x00)

        # Wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
        pi.i2c_write_byte_data(mpu6050_handler, 0x6B, 0x02)

        pi.i2c_write_byte_data(mpu6050_handler, 0x38, 1)

        # Set G Scale
        # Acc_Config = pi.i2c_read_byte_data(mpu6050_handler,0x1C)
        # Acc_Config_4G = (Acc_Config | 1<<3) & (~1<<4)

        self.mpu6050_handle = mpu6050_handler
        print("IMU setup complete")

    def calculate_angles(self, pi):

        # Get accelerometer and gyroscope data
        self.accel_data = self.update_accelerometer_data(pi) - self.acc_offsets
        self.gyro_data = self.update_gyroscope_data(pi) - self.gyro_offsets

        # Estimate angle from accelerometer
        pitch_acc = np.arctan2(self.accel_data[0], np.sqrt(self.accel_data[1] * self.accel_data[1] + self.accel_data[2]
                                                           * self.accel_data[2])) * -1
        roll_acc = np.arctan2(self.accel_data[1], np.sqrt(self.accel_data[0] * self.accel_data[0] + self.accel_data[2]
                                                          * self.accel_data[2]))

        # Complementary Filter
        acc_angles = np.array([roll_acc * 180 / np.pi, pitch_acc * 180 / np.pi])
        gyro_pr = np.array([self.gyro_data[0], self.gyro_data[1]])

        if self.sample_time < 0:
            self.sample_time = 0

        self.euler_state = (self.alpha * (self.euler_state + self.sample_time * gyro_pr) + (1 - self.alpha) * acc_angles)

    # checks the limitations on each variable to avoid reset windup
    def check_output_limitations(self, a, b, c):

        if a > self.output_max:
            self.I_term[0] -= a - self.output_max
            a = self.output_max
        elif a < self.output_min:
            self.I_term[0] += self.output_min - a
            a = self.output_min

        if b > self.output_max:
            self.I_term[1] -= b - self.output_max
            b = self.output_max
        elif b < self.output_min:
            self.I_term[1] += self.output_min - b
            b = self.output_min

        if c > self.output_max:
            self.I_term[2] -= c - self.output_max
            c = self.output_max
        elif c < self.output_min:
            self.I_term[2] += self.output_min - c
            c = self.output_min

        return np.array([a,b,c])
