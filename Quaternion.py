import numpy as np

# defines Quaternion operations
class Quaternion(object):
    def __init__(self, a, b, c, d):
       self.a = a
       self.b = b
       self.c = c
       self.d = d

    # returns the norm of the quaternion
    def norm(self):
        return np.sqrt(self.a * self.a + self.b * self.b + self.c* self.c + self.d * self.d)

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
    
    # convert the quaternion into a string
    def to_string(self):
        print(self.a, self.b, self.c, self.d)

    # convert the quaternion into an array
    def to_array(self):
        arr = np.array([self.a, self.b, self.c, self.d])
        return arr

#convert from euler state to quaternion

#convert from quaternion to euler state
