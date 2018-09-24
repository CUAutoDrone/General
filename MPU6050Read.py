import pigpio
import time
import numpy as np

MPU6050_ADDR = 0x68

def setupMPU6050(pi):
	#opens connection at I2C bus 1
	mpu6050_handle = pi.i2c_open(1,MPU6050_ADDR,0)

	#Wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
	PWR_MGMT_1 = pi.i2c_read_byte_data(mpu6050_handle,0x6B)
	print(bin(PWR_MGMT_1))
	pi.i2c_write_byte_data(mpu6050_handle,0x6B,0)

	#Set G Scale
	Acc_Config = pi.i2c_read_byte_data(mpu6050_handle,0x1C)
	Acc_Config_4G = (Acc_Config | 1<<3) & (~1<<4)

	#Calculate Offsets
	acc_offsets = get_acceleration_data(pi,mpu6050_handle)

	return mpu6050_handle,acc_offsets


def get_acceleration_data(pi,MPU6050_handle):
	sum_acc_x = 0
	sum_acc_y = 0
	sum_acc_z = 0

	iter_num = 10

	for i in range(0,iter_num):
		AcX_1 =pi.i2c_read_byte_data(MPU6050_handle,0x3B)
		AcX_2 = pi.i2c_read_byte_data(MPU6050_handle,0x3C)
		AcX = (AcX_1 << 8) + AcX_2
		sum_acc_x+=AcX

		AcY_1 =pi.i2c_read_byte_data(MPU6050_handle,0x3D)
		AcY_2 = pi.i2c_read_byte_data(MPU6050_handle,0x3E)
		AcY = (AcY_1 << 8) + AcY_2
		sum_acc_y+= AcY

		AcZ_1 =pi.i2c_read_byte_data(MPU6050_handle,0x3F)
		AcZ_2 = pi.i2c_read_byte_data(MPU6050_handle,0x40)
		AcZ = (AcZ_1 << 8) + AcZ_2
		sum_acc_z+= AcZ

	AcX_mean = sum_acc_x/iter_num
	AcY_mean = sum_acc_y/iter_num
	AcZ_mean = sum_acc_z/iter_num

	#Convert to G's
	AcX_mean = AcX_mean/65535*4
	AcY_mean = AcY_mean/65535*4
	AcZ_mean = AcZ_mean/65535*4

	return np.array([AcX_mean,AcY_mean,AcZ_mean])


#Setup
pi = pigpio.pi()
MPU6050_handle,acc_offsets = setupMPU6050(pi)
whoami = pi.i2c_read_byte_data(MPU6050_handle,0x75)


#Loop
while(True):
	accel_data = get_acceleration_data(pi,MPU6050_handle)-acc_offsets
	print(accel_data)
	time.sleep(0.5)
