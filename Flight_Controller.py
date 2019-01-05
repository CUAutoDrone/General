import pigpio
import time
import numpy as np
 #PIN DESIGNATIONS
RECIEVER_CH1 = 17 #GPIO 11
RECIEVER_CH2 = 27 #GPIO 13
RECIEVER_CH3 = 22 #GPIO 15
RECIEVER_CH4 = 18 #GPIO 12
RECIEVER_CH5 = 23 #GPIO 23
MOTOR1 = 10 #GPIO 19
MOTOR2 = 9 #GPIO 21
MOTOR3 = 25 #GPIO 22
MOTOR4 = 8 #GPIO 24
 #PID GAINS (roll, pitch, yaw rate)
Kp = np.array([0.05,0.05,0.05])
Ki = np.array([0,0,0])
Kd = np.array([0,0,0])
 #MPU6050 REGISTERS
MPU6050_ADDR = 0x68
 #GLOBAL VARIABLES FOR PWM MEASUREMENT
rising_1 = 0
pulse_width_ch1 = 0
rising_2 = 0
pulse_width_ch2 = 0
rising_3 = 0
pulse_width_ch3 = 0
rising_4 = 0
pulse_width_ch4= 0
rising_5 = 0
pulse_width_ch5= 0
 ARM = 0
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
 def calculate_angles(pi,accel_data,gyro_data,dt,euler_state):
	#alpha
	alpha = 0.98
 	#convert to radians
	accel_data_rad = accel_data*np.pi/180
 	#Estimate angle from accelerometer
	roll_acc = np.arctan2(accel_data_rad[1],-1*accel_data_rad[2])
	pitch_acc = np.arctan2(-accel_data_rad[0],np.sqrt(np.power(accel_data_rad[1],2)+np.power(accel_data_rad[2],2)))
 	#Complimenatry Filter
	acc_angles = np.array([roll_acc*180/np.pi,pitch_acc*180/np.pi])
	gyro_pr = np.array([gyro_data[0],gyro_data[1]])
 	new_angles = alpha*(euler_state+dt*gyro_pr) + (1-alpha)*acc_angles
 	return new_angles
 def cbf1(gpio,level,tick):
	#connects global and local variables
	global rising_1
	global pulse_width_ch1

	#If rising edge, store time
	if level == 1:
		rising_1 = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
 		width = tick-rising_1
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_ch1 = width/1000
 def cbf2(gpio,level,tick):
	#connects global and local variables
	global rising_2
	global pulse_width_ch2

	#If rising edge, store time
	if level == 1:
		rising_2 = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
 		width = tick-rising_2
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_ch2 = width/1000		
 def cbf3(gpio,level,tick):
	#connects global and local variables
	global rising_3
	global pulse_width_ch3

	#If rising edge, store time
	if level == 1:
		rising_3 = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
 		width = tick-rising_3
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_ch3 = width/1000	
 def cbf4(gpio,level,tick):
	#connects global and local variables
	global rising_4
	global pulse_width_ch4

	#If rising edge, store time
	if level == 1:
		rising_4 = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
 		width = tick-rising_4
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_ch4 = width/1000	
 def cbf5(gpio,level,tick):
	#connects global and local variables
	global rising_5
	global pulse_width_ch5
	global ARM

	#If rising edge, store time
	if level == 1:
		rising_5 = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
 		width = tick-rising_5
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_ch5 = width/1000	
			if pulse_width_ch5 > 1.4:
				ARM = 1
			else:
				ARM = 0
 def map(num,a,b,c,d):
	y = (num-a)*(d-c)/(b-a)+c
	return y
 def map_control_input():
	global pulse_width_ch1
	global pulse_width_ch2
	global pulse_width_ch3
	global pulse_width_ch4
	global pulse_width_ch5
 	roll = map(pulse_width_ch2,1,2,-30,30)
	pitch  = map(pulse_width_ch4,1,2,-30,30)
	yaw = map(pulse_width_ch1,1,2,-10,10)
	throttle = pulse_width_ch3
	arm = pulse_width_ch5
 	return np.array([roll, pitch, yaw, throttle, arm])
 def map_motor_output(ctrl):
	throttle  = ctrl[0]
	roll = ctrl[1]
	pitch = ctrl[2]
	yawrate = ctrl[3]
 	m1 = throttle-roll+pitch-yawrate
	m2 = throttle-roll-pitch+yawrate
	m3 = throttle+roll-pitch-yawrate
	m4 = throttle+roll+pitch+yawrate
	return minMax(np.array([m1,m2,m3,m4]),1,2)
 def minMax(vec,min,max):
	for i in range(0,np.size(vec)):
		if vec[i]<min:
			vec[i] = min
		elif vec[i]>max:
			vec[i] = max	
	return vec
 def set_motor_pulse(pi,gpio, timems):
	pi.set_PWM_dutycycle(gpio,timems/2.5*255)
 def can_arm():
	canarm = True
	if pulse_width_ch3 > 1.0:
		canarm = False
		print("Failed Preflight Check.  Throttle Not Zero")
 	return canarm
 def arm(pi):
	print("Arming...")
	set_motor_pulse(pi,MOTOR1,1)
	set_motor_pulse(pi,MOTOR2,1)
	set_motor_pulse(pi,MOTOR3,1)
	set_motor_pulse(pi,MOTOR4,1)
 #--------------------------------------
#SETUP
pi = pigpio.pi()
 #set reciever input pins 
pi.set_mode(RECIEVER_CH1,pigpio.INPUT)
pi.set_mode(RECIEVER_CH2,pigpio.INPUT)
pi.set_mode(RECIEVER_CH3,pigpio.INPUT)
pi.set_mode(RECIEVER_CH4,pigpio.INPUT)
pi.set_mode(RECIEVER_CH5,pigpio.INPUT)
 #initialize callbacks
cb1 = pi.callback(RECIEVER_CH1, pigpio.EITHER_EDGE,cbf1)
cb2 = pi.callback(RECIEVER_CH2, pigpio.EITHER_EDGE,cbf2)
cb3 = pi.callback(RECIEVER_CH3, pigpio.EITHER_EDGE,cbf3)
cb4 = pi.callback(RECIEVER_CH4, pigpio.EITHER_EDGE,cbf4)
cb5 = pi.callback(RECIEVER_CH5, pigpio.EITHER_EDGE,cbf5)
 #set motor output pins
pi.set_mode(MOTOR1,pigpio.OUTPUT)
pi.set_mode(MOTOR2,pigpio.OUTPUT)
pi.set_mode(MOTOR3,pigpio.OUTPUT)
pi.set_mode(MOTOR4,pigpio.OUTPUT)
pi.set_PWM_frequency(MOTOR1,400)
pi.set_PWM_frequency(MOTOR2,400)
pi.set_PWM_frequency(MOTOR3,400)
pi.set_PWM_frequency(MOTOR4,400)
 #setup IMU
MPU6050_handle,acc_offsets,gyro_offsets = setupMPU6050(pi)
euler_state = np.array([0,0])
 #Machine Loop
while(True):
 	#send zero signal to motors
	set_motor_pulse(pi,MOTOR1,1)
	set_motor_pulse(pi,MOTOR2,1)
	set_motor_pulse(pi,MOTOR3,1)
	set_motor_pulse(pi,MOTOR4,1)
 	#wait for arm
	armed= 0
	while(armed==0):
		if ARM == 1:
			#perform preflight checks
			canarm = can_arm()
			if canarm:
				arm(pi)
				armed=1
 	#Initialize PID Control Stuff
	is_first_loop = 0
	err_sum = 0
	sys_time = pi.get_current_tick()
 	#flight loop
	while(ARM == 1):
 		#Get Delta Time
		sys_time_new = pi.get_current_tick()
		dt = (sys_time_new-sys_time)/1e6
		#correct for rollover
		if dt<0:
			dt=0
		sys_time = sys_time_new
 		# #Maps control input into angles
		control_angles = map_control_input()
 		#Get accelerometer and gyroscope data and compute angles
		accel_data = get_acceleration_data(pi,MPU6050_handle)-acc_offsets
		gyro_data = get_gyroscope_data(pi,MPU6050_handle)-gyro_offsets
		euler_state = calculate_angles(pi,accel_data,gyro_data,dt,euler_state)
 		#Compute errors in pitch and roll and yawrate
		#err = np.array([control_angles[0]-euler_state[0],
		# 				control_angles[1]-euler_state[1],
		# 				control_angles[2]-gyro_data[2]])
		err = np.array([0-euler_state[0],
		 				0-euler_state[1],
		 				0])
 		#compute error integral
		err_sum = err_sum+err
		#PID law
		if is_first_loop == 0:
		 	u = np.multiply(Kp,err)+np.multiply(Ki,dt*err_sum)
			is_first_loop = 1
		else:
			u = np.multiply(Kp,err)+np.multiply(Ki,dt*err_sum)+np.multiply(Kd,(err-prev_err)/dt)
		prev_err = err
 		#Map controls into vector
		ctrl = np.array([control_angles[3],u[0],u[1],u[2]])
 		#Map control angles into output signal and set motors
		wm = map_motor_output(ctrl)
		print(wm)
		set_motor_pulse(pi,MOTOR1,wm[0])
		set_motor_pulse(pi,MOTOR2,wm[1])
		set_motor_pulse(pi,MOTOR3,wm[2])
		set_motor_pulse(pi,MOTOR4,wm[3])
