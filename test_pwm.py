#This program runs the motor connected to the ESC from the RPi.
#The ESC must be connected to the motor and powered with 5.5V.  See Nikhil if you do not know how to do this
#The signal pin of the ESC must be connected to BCM 4 of the RPi.  
#The ground pin of the ESC must be connected to ground on the RPi.
#the command 'sudo pigpiod' must be run before this program will work.

import pigpio
import time

#Instantiates Pi object in pigpio
pi = pigpio.pi()

#sets BCM pin 4 to an output - this is required to produce a signal
pi.set_mode(4,pigpio.OUTPUT)

#Sets the duty cycle to 12.5%
pi.set_PWM_dutycycle(4,12.5)
#Sets frequency to 50 Hz
pi.set_PWM_frequency(4,50)
time.sleep(3)
pi.set_PWM_dutycycle(4,19.1)
time.sleep(3)
pi.set_PWM_dutycycle(4,0)
pi.stop()
