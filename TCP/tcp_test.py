import socket
import pigpio
import time
import numpy as np

HOST = '192.168.2.5'
PORT = 9995

#MPU6050 REGISTERS
MPU6050_ADDR = 0x68


def setupMPU6050(pi):
	#opens connection at I2C bus 1
	mpu6050_handle = pi.i2c_open(1,MPU6050_ADDR,0)

	#Wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
	pi.i2c_write_byte_data(mpu6050_handle, 0x6B, 0x02)

	pi.i2c_write_byte_data(mpu6050_handle, 0x38, 1)

	#Set G Scale
	#Acc_Config = pi.i2c_read_byte_data(mpu6050_handle,0x1C)
	#Acc_Config_4G = (Acc_Config | 1<<3) & (~1<<4)

	#Calculate Offsets
	acc_offsets = get_acc_offsets(pi,mpu6050_handle)
	gyro_offsets = get_gyro_offsets(pi,mpu6050_handle)

	return mpu6050_handle,acc_offsets,gyro_offsets

def get_acc_offsets(pi,MPU6050_handle):
	sum_acc_x = 0
	sum_acc_y = 0
	sum_acc_z = 0

	iter_num = 100

	for i in range(0,iter_num):
		AcX = (pi.i2c_read_byte_data(MPU6050_handle,0x3B) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3C)
		AcY = (pi.i2c_read_byte_data(MPU6050_handle,0x3D) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3E)
		AcZ = (pi.i2c_read_byte_data(MPU6050_handle,0x3F) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x40)
		if AcX > 32768:
			AcX = AcX-65536
		if AcY > 32768:
			AcY = AcY-65536
		if AcZ > 32768:
			AcZ = AcZ-65536

		sum_acc_x += AcX
		sum_acc_y+= AcY
		sum_acc_z+= AcZ

	AcX_mean = sum_acc_x/iter_num
	AcY_mean = sum_acc_y/iter_num
	AcZ_mean = sum_acc_z/iter_num

	#Convert to G's
	AcX_mean = AcX_mean/65535*4
	AcY_mean = AcY_mean/65535*4
	AcZ_mean = AcZ_mean/65535*4

	return np.array([AcX_mean,AcY_mean,AcZ_mean+1])



def get_acceleration_data(pi,MPU6050_handle):

	AcX = (pi.i2c_read_byte_data(MPU6050_handle,0x3B) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3C)
	AcY = (pi.i2c_read_byte_data(MPU6050_handle,0x3D) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3E)
	AcZ = (pi.i2c_read_byte_data(MPU6050_handle,0x3F) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x40)
	if AcX > 32768:
		AcX = AcX-65536
	if AcY > 32768:
		AcY = AcY-65536
	if AcZ > 32768:
		AcZ = AcZ-65536

	#Convert to G's
	AcX = AcX*4/65536
	AcY = AcY*4/65536
	AcZ = AcZ*4/65536


	return np.array([AcX,AcY,AcZ])

def get_gyro_offsets(pi,MPU6050_handle):
	sum_gy_x = 0
	sum_gy_y = 0
	sum_gy_z = 0

	iter_num = 100

	for i in range(0,iter_num):
		GyX = (pi.i2c_read_byte_data(MPU6050_handle,0x43) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x44)
		GyY = (pi.i2c_read_byte_data(MPU6050_handle,0x45) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x46)
		GyZ = (pi.i2c_read_byte_data(MPU6050_handle,0x47) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x48)
		if GyX > 32768:
			GyX = GyX-65536
		if GyY > 32768:
			GyY = GyY-65536
		if GyZ > 32768:
			GyZ = GyZ-65536

		sum_gy_x+= GyX
		sum_gy_y+= GyY
		sum_gy_z+= GyZ

	GyX_mean = sum_gy_x/iter_num
	GyY_mean = sum_gy_y/iter_num
	GyZ_mean = sum_gy_z/iter_num

	#Convert to G's
	GyX_mean = GyX_mean/65535*500
	GyY_mean = GyY_mean/65535*500
	GyZ_mean = GyZ_mean/65535*500

	return np.array([GyX_mean,GyY_mean,GyZ_mean])

def get_gyroscope_data(pi,MPU6050_handle):

	GyX = (pi.i2c_read_byte_data(MPU6050_handle,0x43) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x44)
	GyY = (pi.i2c_read_byte_data(MPU6050_handle,0x45) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x46)
	GyZ = (pi.i2c_read_byte_data(MPU6050_handle,0x47) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x48)
	if GyX > 32768:
		GyX = GyX-65536
	if GyY > 32768:
		GyY = GyY-65536
	if GyZ > 32768:
		GyZ = GyZ-65536

	#Convert to deg/sec
	GyX = GyX*500/65536
	GyY = GyY*500/65536
	GyZ = GyZ*500/65536


	return np.array([GyX,GyY,GyZ])

#setup IMU
pi = pigpio.pi()
MPU6050_handle,acc_offsets,gyro_offsets = setupMPU6050(pi)
euler_state = np.array([0,0])

#Start Server and send data
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((HOST,PORT))
	s.listen()
	conn,addr = s.accept()
	with conn:
		print('Connected by',addr)
		while True:
			accel_data = get_acceleration_data(pi,MPU6050_handle)
			ax = str(np.around(accel_data[1],3)).ljust(8)
			conn.sendall(ax.encode("utf-8"))
		