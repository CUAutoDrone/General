import pigpio
import time

#global variable for time at rising edge
rising = 0
#global variable for pulse width
pulse_width = 0

def cbf(gpio,level,tick):
	#connects global and local variables
	global rising
	global pulse_width
	
	#If rising edge, store time
	if level == 1:
		rising = tick
	#If fallign edge, subtract out rising time to get pusle width
	elif level == 0:
		
		
		pulse_width = tick-rising
		#in case of wraparound
		if pulse_width>0:
			#divide by 1000 to get milliseconds
			# pulse_width = pulse_width/1000
			
			#divide by 147 as conversion to inches (147 micro-secs per inch)
			pulse_width = pulse_width/147
			
		#call angle function, get angle offset, do the math
		#cos(angle) * pulse_width is the new pulse width

pi = pigpio.pi()
#sets BCM 4 to input
pi.set_mode(4,pigpio.INPUT)
#initializes callback, either logic change
cb1 = pi.callback(4, pigpio.EITHER_EDGE,cbf)

while True:
	print(pulse_width)
	time.sleep(0.03)
