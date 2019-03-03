import pigpio
import time

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
        
        #in case of wraparound
        width = tick-rising

        #Divide by 147, conversion factor is 147 uS to 1 inch
        pw_inches = width/147

pi = pigpio.pi()
#sets BCM 5 to input
pi.set_mode(18,pigpio.INPUT)
#initializes callback, either logic change
cb1 = pi.callback(18, pigpio.EITHER_EDGE,cbf)

while True:
    print(str (pw_inches) + " inches away")
    time.sleep(0.03)
