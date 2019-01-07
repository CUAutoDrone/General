import numpy as np
import math


# defines Quaternion operations
class Quaternion(object):
    def __init__(self, a, b, c, d):
       self.a = a
       self.b = b
       self.c = c
       self.d = d

    # returns the norm of the quaternion
    def norm(self):
        return np.sqrt(self.a * self.a + self.b * self.b + self.c * self.c + self.d * self.d)

    # returns the conjugate of the quaternion
    def conjugate(self):
        self.a = self.a * -1
        self.b = self.b * -1
        self.c = self.c * -1
        self.d = self.d * -1

    # add two quaternions
    def plus(self, q):
        self.a = self.a + q.a
        self.b = self.b + q.b
        self.c = self.c + q.c
        self.d = self.d + q.d

    # multiply two quaternions
    def times(self, q):
        d = self.a * q.a - self.b * q.b - self.c * q.c - self.d * q.d
        e = self.a * q.b + self.b * q.a + self.c * q.d - self.d * q.c
        f = self.a * q.c - self.b * q.d + self.c * q.a + self.d * q.b
        g = self.a * q.d + self.b * q.c - self.c * q.b + self.d * q.a
        self.a = d
        self.b = e
        self.c = f
        self.d = g

    # inverse of the quaternion
    def inverse(self):
        add = self.a * self.a + self.b * self.b + self.c * self.c + self.d * self.d
        self.a = self.a / add
        self.b = self.b / add
        self.c = self.c / add
        self.d = self.d / add

    # divide two quaternions
    def divided_by(self, q):
        q.inverse()
        self.times(q)

    def to_string(self):
        print(self.a, self.b, self.c, self.d)

    def to_array(self):
        arr = np.array([self.a, self.b, self.c, self.d])
        return arr

    # convert from euler state to quaternion
    # TODO: more thorough testing
    @staticmethod
    def euler_angle_to_quaternion(angle_x, angle_y, angle_z):
        heading = (angle_y / 180) * math.pi
        attitude = (angle_z / 180) * math.pi
        bank = (angle_x / 180) * math.pi

        c1 = math.cos(heading / 2)
        c2 = math.cos(attitude / 2)
        c3 = math.cos(bank / 2)

        s1 = math.sin(heading / 2)
        s2 = math.sin(attitude / 2)
        s3 = math.sin(bank / 2)

        a = (c1 * c2 * c3) - (s1 * s2 * s3)
        b = (s1 * s2 * c3) + (c1 * c2 * s3)
        c = (s1 * c2 * c3) + (c1 * s2 * s3)
        d = (c1 * s2 * c3) - (s1 * c2 * s3)

        return Quaternion(a, b, c, d)

    # convert from quaternion to euler angle
    #TODO: Taken from General/BNO055Test.py, have NOT tested
    @staticmethod
    def quaternion_to_euler_angle(a, b, c, d):
        t0 = +2.0 * (a * b + c * d)
        t1 = +1.0 - 2.0 * (b * b + c * c)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (a * c - d * b)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (a * d + b * c)
        t4 = +1.0 - 2.0 * (c * c + d * d)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z


