import pigpio
import time
import numpy as np
import atexit
import csv
from KalmanAttitude import KalmanAttitude

 #PIN DESIGNATIONS
RECIEVER_CH1 = 17 #GPIO 11
RECIEVER_CH2 = 27 #GPIO 13
RECIEVER_CH3 = 22 #GPIO 15
RECIEVER_CH4 = 18 #GPIO 12
RECIEVER_CH5 = 11 #GPIO 23
MOTOR1 = 10 #GPIO 19
MOTOR2 = 9 #GPIO 21
MOTOR3 = 25 #GPIO 22
MOTOR4 = 8 #GPIO 24

#MPU6050 REGISTERS
MPU6050_ADDR = 0x68
AK8963_ADDR = 0x0C
AK8963_CONFIG_ADDR = 0x0A
CONFIG = 0x1A

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
AUTOARM = 1

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
            if pulse_width_ch1 < 0.1 or pulse_width_ch1 > 3:
                print('Lost Reciever Channel 1')
                AUTOARM = 0

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
            if pulse_width_ch2 < 0.1 or pulse_width_ch2 > 3:
                print('Lost Reciever Channel 2')
                AUTOARM = 0     

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
            if pulse_width_ch3 < 0.1 or pulse_width_ch3 > 3:
                print('Lost Reciever Channel 3')
                AUTOARM = 0 

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
            if pulse_width_ch4 < 0.1 or pulse_width_ch4 > 3:
                print('Lost Reciever Channel 4')
                AUTOARM = 0 

def cbf5(gpio,level,tick):

    #connects global and local variables
    global rising_5
    global pulse_width_ch5
    global ARM
    global AUTOARM
    
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
            if pulse_width_ch5 > 1.4 and pulse_width_ch5<2 and ARM == 0:
                ARM = 1
                AUTOARM = 1
            elif pulse_width_ch5< 1.4 or pulse_width_ch5>2:
                ARM = 0

def setupMPU6050(pi):
    #opens connection at I2C bus 1
    mpu6050_handle = pi.i2c_open(1,MPU6050_ADDR,0)

    #Wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
    pi.i2c_write_byte_data(mpu6050_handle, 0x6B, 0x02)
    pi.i2c_write_byte_data(mpu6050_handle, 0x38, 1)
    pi.i2c_write_byte_data(mpu6050_handle,0x1A,0x03)
    time.sleep(0.1)

    #Set G Scale
    #Acc_Config = pi.i2c_read_byte_data(mpu6050_handle,0x1C)
    #Acc_Config_4G = (Acc_Config | 1<<3) & (~1<<4)

    #Calculate Offsets
    acc_offsets = get_acc_offsets(pi,mpu6050_handle)
    gyro_offsets = get_gyro_offsets(pi,mpu6050_handle)

    #magnetometer
    bypass_byte = (pi.i2c_read_byte_data(mpu6050_handle,0x37))|(0b00000010)
    pi.i2c_write_byte_data(mpu6050_handle,0x37,bypass_byte)
    pi.i2c_write_byte_data(mpu6050_handle,0x38,0x01)
    ak8960_handle = pi.i2c_open(1,AK8963_ADDR,0)

    pi.i2c_write_byte_data(ak8960_handle,AK8963_CONFIG_ADDR,0b00010110)
    time.sleep(0.1)

    return mpu6050_handle,ak8960_handle,acc_offsets,gyro_offsets

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

    return np.array([AcX_mean-4/5,AcY_mean-3/5,AcZ_mean])

def get_acceleration_data(pi,MPU6050_handle):

    try:
        AcX = (pi.i2c_read_byte_data(MPU6050_handle,0x3B) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3C)
        AcY = (pi.i2c_read_byte_data(MPU6050_handle,0x3D) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x3E)
        AcZ = (pi.i2c_read_byte_data(MPU6050_handle,0x3F) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x40)
    except:
        print('Lost IMU Connection')
        AUTOARM = 0

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
    GyX_mean = GyX_mean/65.5
    GyY_mean = GyY_mean/65.5
    GyZ_mean = GyZ_mean/65.5

    return np.array([GyX_mean,GyY_mean,GyZ_mean])

def get_gyroscope_data(pi,MPU6050_handle):
    try:
        GyX = (pi.i2c_read_byte_data(MPU6050_handle,0x43) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x44)
        GyY = (pi.i2c_read_byte_data(MPU6050_handle,0x45) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x46)
        GyZ = (pi.i2c_read_byte_data(MPU6050_handle,0x47) << 8) + pi.i2c_read_byte_data(MPU6050_handle,0x48)
    except:
        AUTOARM == 0

    if GyX > 32768:
        GyX = GyX-65536
    if GyY > 32768:
        GyY = GyY-65536
    if GyZ > 32768:
        GyZ = GyZ-65536

    #Convert to deg/sec
    GyX = GyX/65535*500
    GyY = GyY/65535*500
    GyZ = GyZ/65535*500


    return np.pi*np.array([GyX,GyY,GyZ])/180

def get_magnetometer_data(pi,ak8960_handle):
    ctrl1 = pi.i2c_read_byte_data(ak8960_handle,0x02)
    mx = (pi.i2c_read_byte_data(ak8960_handle,0x04) << 8) + pi.i2c_read_byte_data(ak8960_handle,0x03)
    my = (pi.i2c_read_byte_data(ak8960_handle,0x06) << 8) + pi.i2c_read_byte_data(ak8960_handle,0x05)
    mz = (pi.i2c_read_byte_data(ak8960_handle,0x08) << 8) + pi.i2c_read_byte_data(ak8960_handle,0x07)
    ctrl2 = pi.i2c_read_byte_data(ak8960_handle,0x09)
    if mx > 32768:
        mx = mx-65536
    if my > 32768:
        my = my-65536
    if mz > 32768:
        mz = mz-65536

    return np.array([mx,my,mz])

def quat2euler(q):
    roll = np.arctan2(2*(q[0,0]*q[0,1]+q[0,2]*q[0,3]),1-2*(q[0,1]**2+q[0,2]**2))
    pitch = np.arcsin(2*(q[0,0]*q[0,2]-q[0,3]*q[0,1]))
    yaw = np.arctan2(2*(q[0,0]*q[0,3]+q[0,1]*q[0,2]),1-2*(q[0,2]**2+q[0,3]**2))
    return 180/np.pi*np.array([roll,pitch,yaw])

def qtoDCM(q):
    C = np.array([[q[0]**2+q[1]**2-q[2]**2-q[3]**2, 2*q[0]*q[3]+2*q[1]*q[2], -2*q[0]*q[2]+2*q[1]*q[3]],
    [-2*q[0]*q[3]+2*q[1]*q[2], q[0]**2-q[1]**2+q[2]**2-q[3]**2, 2*q[0]*q[1]+2*q[2]*q[3]],
    [2*q[0]*q[2]+2*q[1]*q[3], -2*q[0]*q[1]+2*q[2]*q[3], q[0]**2-q[1]**2-q[2]**2+q[3]**2]])
    return C

def quatmult(p,r):
    q0 = p[0,0]*r[0,0]-p[0,1]*r[0,1]-p[0,2]*r[0,2]-p[0,3]*r[0,3]
    q1 = p[0,0]*r[0,1]+p[0,1]*r[0,0]+p[0,2]*r[0,3]-p[0,3]*r[0,2]
    q2 = p[0,0]*r[0,2]-p[0,1]*r[0,3]+p[0,2]*r[0,0]+p[0,3]*r[0,1]
    q3 = p[0,0]*r[0,3]+p[0,1]*r[0,2]-p[0,2]*r[0,1]+p[0,3]*r[0,0]
    return np.array([[q0,q1,q2,q3]])

def quatinverse(q):
    q_out = np.array([[q[0,0], -q[0,1],-q[0,2],-q[0,3]]])/np.linalg.norm(q);
    return q_out


def map(num,a,b,c,d):
    y = (num-a)*(d-c)/(b-a)+c
    return y

def minMax(vec,min,max):
    for i in range(0,np.size(vec)):
        if vec[i]<min:
            vec[i] = min
        elif vec[i]>max:
            vec[i] = max    
    return vec



def map_control_input():
    global pulse_width_ch1
    global pulse_width_ch2
    global pulse_width_ch3
    global pulse_width_ch4
    global pulse_width_ch5

    roll = map(pulse_width_ch4,1,2,-30,30)
    pitch  = map(pulse_width_ch2,1,2,-30,30)
    yaw = map(pulse_width_ch1,1,2,-10,10)
    throttle = map(pulse_width_ch3,1,2,0,13)
    arm = pulse_width_ch5

    return np.array([roll, pitch, yaw, throttle, arm])

def control_into_qref(ctrl):
    roll = np.pi*ctrl[0]/180
    pitch = np.pi*ctrl[1]/180
    yaw = 0
    croll = np.cos(roll/2)
    sroll = np.sin(roll/2)
    cpitch = np.cos(pitch/2)
    spitch = np.sin(pitch/2)
    cyaw = np.cos(yaw/2)
    syaw = np.sin(yaw/2)
    q0 = croll*cpitch*cyaw+sroll*spitch*syaw
    q1 = sroll*cpitch*cyaw-croll*spitch*syaw
    q2 = croll*spitch*cyaw+sroll*cpitch*syaw
    q3 = croll*cpitch*syaw-sroll*spitch*cyaw
    return np.array([[q0,q1,q2,q3]])


def map_motor_output(throttle,tx,ty,km,l,pw0):
    pw1 = 0
    pw2 = 0
    pw3 = 0
    pw4 = 0
    if tx>=0 and ty>=0:
        pw4 = pw4+(tx+ty)/(2*km*l)
        if tx-km*l*pw4>0:
            pw3 = pw3+(tx-km*l*pw4)/(km*l)
        else:
            pw1 = pw1+(ty-km*l*pw4)/(km*l)
    elif tx>=0 and ty<0:
        pw3 = pw4+(tx-ty)/(2*km*l)
        if tx-km*l*pw3>0:
            pw4 = pw4+(tx-km*l*pw3)/(km*l)
        else:
            pw2 = pw2+(-ty-km*l*pw3)/(km*l)
    elif tx<0 and ty>=0:
        pw1 = pw1+(-tx+ty)/(2*km*l)
        if -tx-km*l*pw1>0:
            pw2 = pw2+(-tx-km*l*pw1)/(km*l)
        else:
            pw2 = pw2+(ty-km*l*pw1)/(km*l)
    elif tx<0 and ty<0:
        pw2 = pw2+(-tx-ty)/(2*km*l)
        if -tx-km*l*pw2>0:
            pw1 = pw1+(-tx-km*l*pw2)/(km*l)
        else:
            pw3 = pw3+(-ty-km*l*pw2)/(km*l)

    res_thrust = throttle-km*(pw1+pw2+pw3+pw4)
    if res_thrust>0:
        pw1 = pw1+res_thrust/(4*km);
        pw2 = pw2+res_thrust/(4*km);
        pw3 = pw3+res_thrust/(4*km);
        pw4 = pw4+res_thrust/(4*km);


    return minMax(np.array([pw1+pw0,pw2+pw0,pw3+pw0,pw4+pw0]),1,2)

def set_motor_pulse(pi,gpio, timems):
    pi.set_PWM_dutycycle(gpio,timems/2.5*255)

def can_arm():
    canarm = True
    if pulse_width_ch3 > 1.0:
        canarm = False
        print("Failed Preflight Check.  Throttle Not Zero")

    return canarm

def check_rec_heartbeat(sys_time):
    global AUTOARM

    heart_time_1 = (sys_time - rising_1)/1000
    heart_time_2 = (sys_time - rising_2)/1000
    heart_time_3 = (sys_time - rising_3)/1000
    heart_time_4 = (sys_time - rising_4)/1000
    heart_time_5 = (sys_time - rising_5)/1000

    if heart_time_1 >100 or heart_time_2>100 or heart_time_3>100 or heart_time_4>100 or heart_time_5>100:
        print('Lost Reciever')
        AUTOARM = 0

def arm(pi):
    print("Arming...")
    set_motor_pulse(pi,MOTOR1,1)
    set_motor_pulse(pi,MOTOR2,1)
    set_motor_pulse(pi,MOTOR3,1)
    set_motor_pulse(pi,MOTOR4,1)

def kill_on_crash(pi):
    print('Crashed! :(')
    set_motor_pulse(pi,MOTOR1,1)
    set_motor_pulse(pi,MOTOR2,1)
    set_motor_pulse(pi,MOTOR3,1)
    set_motor_pulse(pi,MOTOR4,1)


#--------------------------------------
#SETUP
pi = pigpio.pi()
atexit.register(kill_on_crash,pi)
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
MPU6050_handle,ak8960_handle,acc_offsets,gyro_offsets = setupMPU6050(pi)

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

        sys_time = pi.get_current_tick()
        check_rec_heartbeat(sys_time)

        if ARM == 1 and AUTOARM == 1:
            #perform preflight checks
            canarm = can_arm()
            if canarm:
                arm(pi)
                armed=1

    #Initialize  Control Stuff
    is_first_loop = 0
    err_sum = 0
    q_old = np.array([[1,0,0,0]])
    P_old = np.diag(np.array([100,100,100,100]))
    sys_time = pi.get_current_tick()
    #magcal = np.array([75.5,181.5,-122.5])
    magcal = np.array([210,-70,-200])
    sys_time = pi.get_current_tick()
    K = 0.5*np.array([[13.4164,0,0,1.0425,0,0],[0,13.4164,0,0,1.0425,0],[0,0,13.4164,0,0,1.3803]])
    km = 5.73
    l = 0.123
    pw0 =1.16
    #q_static = np.array([[ 0.57349583, -0.81795304,  0.02150151,  0.02462935]])
    q_static = np.array([[0.5709,-0.8206,0.0157,0.0226]])
    #q_static = np.array([[ 0.57349583, 0.81795304,  -0.02150151,  -0.02462935]])
    first_iter = 1
    cornerfreq = 30

    #start data collection
    file = open('data.csv','w+')

    
    #flight loop
    #while(True):
    while(ARM == 1 and AUTOARM == 1):
        #Get Delta Time
        sys_time_new = pi.get_current_tick()
        dt = (sys_time_new-sys_time)/1e6
        #correct for rollover
        if dt<0:
            dt=0
        sys_time = sys_time_new

        #Check Reciever heartbeat
        check_rec_heartbeat(sys_time)

        # #Maps control input into angles
        control_angles = map_control_input()

        #Get accelerometer and gyroscope data and compute angles
        accel_data_new = get_acceleration_data(pi,MPU6050_handle)
        if first_iter == 1:
            accel_data = accel_data_new
            first_iter = 0
        else:
            alpha = dt/(1/(2*np.pi*cornerfreq)+dt)
            accel_data = (1-alpha)*accel_data_old+alpha*accel_data_new;

        gyro_data = get_gyroscope_data(pi,MPU6050_handle)#-gyro_offsets
        mag_data = get_magnetometer_data(pi,ak8960_handle)
        [q_new,P_new] = KalmanAttitude(accel_data,gyro_data,mag_data,magcal,dt,q_old,P_old)
        #print(quat2euler(q_new))
        #print(accel_data)
        accel_data_old = accel_data
        q_old = q_new
        P_old = P_new

        q_adj = quatmult(q_new,quatinverse(q_static))

        #write q_adj to file
        file.write(str(sys_time)+','+str(q_adj[0,0])+','+str(q_adj[0,1])+','+str(q_adj[0,2])+','+str(q_adj[0,3])+'\n')

        #print(q_adj)
        print(quat2euler(q_adj))

        #LQR Controller
        q_ref_in = control_into_qref(control_angles)
        #print(q_ref_in)
        q_ref = np.array([[1,0,0,0]])
        #q_ref = np.array([[0.9997,0,-0.0262,0]])
        #Compute quaternon error
        q_err = quatmult(q_ref_in, quatinverse(q_adj))
        x_err = np.transpose(np.array([[q_err[0,1],q_err[0,2],q_err[0,3],-gyro_data[1],gyro_data[0],gyro_data[2]]])) 
        tau_in = -np.dot(K,x_err)
        #print(tau_in)
        thrust_desired = control_angles[3]
        motor_output = map_motor_output(thrust_desired,tau_in[0],tau_in[1],km,l,pw0)

        #print(motor_output)
        set_motor_pulse(pi,MOTOR1,motor_output[0])
        set_motor_pulse(pi,MOTOR2,motor_output[1])
        set_motor_pulse(pi,MOTOR3,motor_output[2])
        set_motor_pulse(pi,MOTOR4,motor_output[3])
        # set_motor_pulse(pi,MOTOR1,1.0)
        # set_motor_pulse(pi,MOTOR2,1.0)
        # set_motor_pulse(pi,MOTOR3,1.0)
        # set_motor_pulse(pi,MOTOR4,1.0)

    file.close()



