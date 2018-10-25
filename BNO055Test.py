import numpy as np
import pigpio
import math

BNO055_ADDR = 0x28
OPR_MODE = 0x3D
AXIS_MAP_SIGN = 0x42

def setupBNO055(pi):
	bno055_handle = pi.i2c_open(1,BNO055_ADDR,0)

	#Map z down
	#pi.i2c_write_byte_data(bno055_handle,AXIS_MAP_SIGN,0x01)

	#change config to NDOF, xxxx1100b
	#opr_mode = pi.i2c_read_byte_data(bno055_handle,OPR_MODE)
	#opr_mode_NDOF = (opr_mode | 0b00001100) & (0b11111100)
	pi.i2c_write_byte_data(bno055_handle,OPR_MODE,0x0C)

	return bno055_handle

def _read_vector(pi, bno055_handle,LSBaddress):
        # Read count number of 16-bit signed values starting from the provided
        # address. Returns a tuple of the values that were read.
        LSB = pi.i2c_read_byte_data(bno055_handle, LSBaddress)
        MSB = pi.i2c_read_byte_data(bno055_handle, LSBaddress+1)
        result = (pi.i2c_read_byte_data(bno055_handle,LSBaddress+1) << 8) | pi.i2c_read_byte_data(bno055_handle,LSBaddress)
        if result > 32767:
            result -= 65536
        return result


def get_euler(pi,bno055_handle):
	roll = (pi.i2c_read_byte_data(bno055_handle,0x1D) << 8) | pi.i2c_read_byte_data(bno055_handle,0x1C)
	if roll > 32767:
		roll = roll- 65536


	return roll/16

def get_quaternion(pi,bno055_handle):
	w = _read_vector(pi,bno055_handle,0x20)
	x = _read_vector(pi,bno055_handle,0x22)
	y = _read_vector(pi,bno055_handle,0x24)
	z = _read_vector(pi,bno055_handle,0x26)
	
	return np.array([w,x,y,z])

def quaternion_to_euler_angle(w, x, y, z):
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z


pi = pigpio.pi()
bno055_handle = setupBNO055(pi)


while(True):
	roll = get_euler(pi,bno055_handle)
	#q = get_quaternion(pi,bno055_handle)
	#roll,pitch,yaw = quaternion_to_euler_angle(q[0],q[1],q[2],q[3])
	heading, roll, pitch = bno.read_euler()

	print("Roll: "+ str(roll))
