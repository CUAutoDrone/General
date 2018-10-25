import pigpio
import time

#Instantiates Pi object in pigpio
pi = pigpio.pi()

#sets BCM pin 4 to an output - this is required to produce a signal
pi.set_mode(4,pigpio.OUTPUT)

#Sets the duty cycle to 12.5%
pi.set_PWM_dutycycle(4,1.3/20*255)
#Sets frequency to 50 Hz
pi.set_PWM_frequency(4,50)
time.sleep(3)
pi.set_PWM_dutycycle(4,1/20*255)
pi.stop()