import pigpio
import time

#global variable for time at rising edge
rising = 0
#global variable for pulse width
pulse_width_1 = 0

def cbf(gpio,level,tick):
	#connects global and local variables
	global rising
	global pulse_width_1
	
	#If rising edge, store time
	if level == 1:
		rising = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:

		width = tick-rising
		#in case of wraparound
		if width>0:
			#divide by 1000 to get milliseconds
			pulse_width_1 = width/1000


pi = pigpio.pi()
#sets BCM 4 to input
pi.set_mode(4,pigpio.INPUT)
#initializes callback, either logic change
cb1 = pi.callback(4, pigpio.EITHER_EDGE,cbf)

while True:
	print(pulse_width_1)
	time.sleep(0.03)