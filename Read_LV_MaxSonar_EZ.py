import pigpio
import time
import math
import numpy as np
from KalmanAttitude import KalmanAttitude

#global variable for time at rising edge
rising = 0
#Gloabl variable for pulse width in inches
pw_inches = 0

def cbf(gpio,level,tick):
    #connects global and local variables
    global rising
    global pw_inches
    #If rising edge, store time
    if level == 1:
        rising = tick
    #If fallign edge, subtract out rising time to get pusle width
    elif level == 0:
        
        # in case of wraparound
        width = tick-rising

        # Divide by 147, conversion factor is 147 uS to 1 inch
        # Fix the angle difference.
        pw_inches = qtoDCM(q_old) / (width/147)  # R(q) / b3 = e3

pi = pigpio.pi()
#sets BCM 5 to input
pi.set_mode(5,pigpio.INPUT)
#initializes callback, either logic change
cb1 = pi.callback(5, pigpio.EITHER_EDGE,cbf)

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

def qtoAngle(q):
    R = qtoDCM(q)
    e = np.array([[0,0,-1]])
    angle = np.dot(e,np.mulmat(R,e))
    return angle

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([[-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0]])

def quatmult(p,r):
    q0 = p[0,0]*r[0,0]-p[0,1]*r[0,1]-p[0,2]*r[0,2]-p[0,3]*r[0,3]
    q1 = p[0,0]*r[0,1]+p[0,1]*r[0,0]+p[0,2]*r[0,3]-p[0,3]*r[0,2]
    q2 = p[0,0]*r[0,2]-p[0,1]*r[0,3]+p[0,2]*r[0,0]+p[0,3]*r[0,1]
    q3 = p[0,0]*r[0,3]+p[0,1]*r[0,2]-p[0,2]*r[0,1]+p[0,3]*r[0,0]
    return np.array([[q0,q1,q2,q3]])

def quatinverse(q):
    q_out = np.array([[q[0,0], -q[0,1],-q[0,2],-q[0,3]]])/np.linalg.norm(q);
    return q_out



#Setup
pi = pigpio.pi()
MPU6050_handle,ak8960_handle,acc_offsets,gyro_offsets = setupMPU6050(pi)
whoami = pi.i2c_read_byte_data(MPU6050_handle,0x75)
q_old = np.array([[1,0,0,0]])
P_old = np.diag(np.array([100,100,100,100]))
sys_time = pi.get_current_tick()
magcal = np.array([75.5,181.5,-122.5])
cornerfreq = 50
q_static = np.array([[ 0.57349583, -0.81795304,  0.02150151,  0.02462935]])
#Loop
first_iter= 1;
            
def helper():
    sys_time_new = pi.get_current_tick()
    dt = (sys_time_new-sys_time)/1e6
    #correct for rollover
    if dt<0:
        dt=0
    sys_time = sys_time_new

    accel_data_new = get_acceleration_data(pi,MPU6050_handle)
    #Get accelerometer and gyroscope data and compute angles
    if first_iter == 1:
        accel_data = accel_data_new
        first_iter = 0
    else:
        alpha = dt/(1/cornerfreq+dt)
        accel_data = (1-alpha)*accel_data_old+alpha*accel_data_new;
        
    gyro_data = get_gyroscope_data(pi,MPU6050_handle)#-gyro_offsets
    #print(gyro_data)
    mag_data = get_magnetometer_data(pi,ak8960_handle)
    #euler_state = calculate_angles(pi,accel_data,gyro_data,dt,euler_state)
    #[q_new,P_new] = KalmanAttitude(pi,accel_data,gyro_data,mag_data,dt,q_old,P_old)
    #print(q_new)
    [q_new,P_new] = KalmanAttitude(accel_data,gyro_data,mag_data,magcal,dt,q_old,P_old)
    #print(q_new)
    #print(quat2euler(q_new))
    q_adj = quatmult(q_new,quatinverse(q_static))
    print(quat2euler(q_adj))

    accel_data_old = accel_data
    q_old = q_new
    P_old = P_new
    
    return;

while True:
    helper()
    print(str (pw_inches) + " inches away")
    time.sleep(0.03)
