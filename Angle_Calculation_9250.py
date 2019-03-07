import pigpio
import time
import numpy as np
from KalmanAttitude import KalmanAttitude

MPU6050_ADDR = 0x68
AK8963_ADDR = 0x0C
AK8963_CONFIG_ADDR = 0x0A
CONFIG = 0x1A

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
    #AcX = AcX*4/65536
    #AcY = AcY*4/65536
    #AcZ = AcZ*4/65536

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

def calculate_angles(pi,accel_data_rad,gyro_data,sys_time,euler_state):
    #alpha
    alpha = 0.85

    #convert to body frame

    accel_data_rad = np.array([0.6*accel_data_rad[0]-0.8*accel_data_rad[1],
                    accel_data_rad[2],
                    -0.8*accel_data_rad[0]-0.6*accel_data_rad[1]])
    gyro_data = np.array([0.6*gyro_data[0]-0.8*gyro_data[1],
                    gyro_data[2],
                    -0.8*gyro_data[0]-0.6*gyro_data[1]])

    #Estimate angle from accelerometer
    roll_acc = np.arctan2(accel_data_rad[1],-accel_data_rad[2])
    pitch_acc = np.arctan2(-accel_data_rad[0],np.sqrt(np.power(accel_data_rad[1],2)+np.power(accel_data_rad[2],2)))

    #Complimenatry Filter
    acc_angles = np.array([roll_acc*180/np.pi,pitch_acc*180/np.pi])
    gyro_pr = np.array([gyro_data[0],gyro_data[1]])
    dt = (pi.get_current_tick()-sys_time)/1e6
    if dt<0:
        dt = 0

    new_angles = alpha*(euler_state+dt*gyro_pr) + (1-alpha)*acc_angles

    return new_angles

# def KalmanAttitude(pi,accel,gyro,mag,dt,qin,Pin):
#     a = accel/np.linalg.norm(accel)
#     if a[2] >=0:
#         q_acc = np.array([np.sqrt((a[2]+1)/2), -a[1]/np.sqrt(2*(a[2]+1)),  a[0]/np.sqrt(2*(a[2]+1)),0])
#     else:
#         q_acc = np.array([-a[1]/np.sqrt(2*(1-a[0])), np.sqrt((1-a[2])/2), 0, a[0]/np.sqrt(2*(1-a[2]))])

#     m_raw = mag/np.linalg.norm(mag)
#     m_aligned = np.array([m_raw[1],m_raw[0],m_raw[2]])
#     R_acc = np.transpose(qtoDCM(q_acc))
#     l = R_acc.dot(np.transpose(m_aligned))
#     gam = l[0]**2+l[1]**2
#     if l[0]>=0:
#         q_mag = np.array([np.sqrt(gam+l[0]*np.sqrt(gam))/np.sqrt(2*gam),0,0,l[1]/(np.sqrt(2)*np.sqrt(gam+l[0]*np.sqrt(gam)))])
#     else:
#         q_mag = [l[1]/(np.sqrt(2)*np.sqrt(gam-l[0]*np.sqrt(gam))),0,0,np.sqrt(gam-l[0]*np.sqrt(gam))/np.sqrt(2*gam)]

#     q_est = quaternion_multiply(q_acc,q_mag);
#     #print(quat2euler(q_est))

#     omega = -1/2*np.array([[0,-gyro[0],-gyro[1],-gyro[2]],
#             [gyro[0], 0, gyro[2], -gyro[1]],
#             [gyro[1], -gyro[2], 0, gyro[0]],
#             [gyro[2], gyro[1], -gyro[0], 0]])

#     F = np.identity(4)+1/2*omega*dt;
#     xhat_proj = F.dot(np.transpose(qin))
#     Gk = np.array([[qin[0,1], qin[0,2], qin[0,3]],
#         [-qin[0,0],qin[0,3], -qin[0,2]],
#         [-qin[0,3], -qin[0,0], qin[0,1]],
#         [qin[0,2],-qin[0,1], -qin[0,0]]])

#     GyroCov = np.array([[0.0008, 0, 0],
#             [0, 0.0013, 0],
#             [0, 0, 0.0014]])

#     Qk = (dt**2)/4*Gk.dot(GyroCov.dot(np.transpose(Gk)))
#     R = np.diag(np.array([0.0012,0.0132,0.0525,0.4489]))
#     P_proj = F.dot(Pin.dot(np.transpose(F)))+Qk
#     print(P_proj)
#     Gain = P_proj.dot(np.linalg.inv(P_proj+R))
#     xhat_k1 = xhat_proj + Gain.dot((np.transpose(q_est)-xhat_proj))
#     P_k1 = np.dot((np.identity(4)-Gain),P_proj)

#     q_out = np.transpose(xhat_k1)
#     P_out = P_k1
#     return [q_out,P_out]

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
#q_static = np.array([[ 0.20118842, -0.35939725, -0.73926535,  0.53262364]])


#Loop
first_iter= 1;
while(True):
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
