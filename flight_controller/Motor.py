# a class representing the motors
class Motor(object):
    def __init__(self, MOTOR1, MOTOR2, MOTOR3, MOTOR4):
        self.MOTOR1 = MOTOR1
        self.MOTOR2 = MOTOR2
        self.MOTOR3 = MOTOR3
        self.MOTOR4 = MOTOR4

    @staticmethod
    def map_motor_output(ctrl):
        throttle = ctrl[0]
        roll = ctrl[1]
        pitch = ctrl[2]
        yawrate = ctrl[3]

        m1 = throttle - roll + pitch - yawrate
        m2 = throttle - roll - pitch + yawrate
        m3 = throttle + roll - pitch - yawrate
        m4 = throttle + roll + pitch + yawrate
        return Motor.minMax(np.array([m1, m2, m3, m4]), 1, 2)

    @staticmethod
    def minMax(vec, min, max):
        for i in range(0, np.size(vec)):
            if vec[i] < min:
                vec[i] = min
            elif vec[i] > max:
                vec[i] = max
        return vec

    @staticmethod
    def set_motor_pulse(pi, gpio, timems):
        pi.set_PWM_dutycycle(gpio, timems / 2.5 * 255)

    def arm(self, pi):
        print("Arming...")
        Motor.set_motor_pulse(pi, self.MOTOR1, 1)
        Motor.set_motor_pulse(pi, self.MOTOR2, 1)
        Motor.set_motor_pulse(pi, self.MOTOR3, 1)
        Motor.set_motor_pulse(pi, self.MOTOR4, 1)
