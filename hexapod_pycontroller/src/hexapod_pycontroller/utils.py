import math

class Point:
    def __init__(self, x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z

    def clone(self):
        return Point(self.x, self.y, self.z)

    def set(self, p):
        self.x = p.x
        self.y = p.y
        self.z = p.z

    def __str__(self):
        return "[{},{},{}]".format(self.x, self.y, self.z)

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y, self.z + p.z)

    def __iadd__(self, p):
        self.x += p.x
        self.y += p.y
        self.z += p.z
        return self

    def __sub__(self, p):
        return (self + p*-1)

    def __mul__(self, s):
        return Point(self.x*s, self.y*s, self.z*s) 
    
    def __imul__(self, s):
        self.x *= s
        self.y *= s
        self.z *= s
        return self

    def __div__(self, s):
        return self * (1/s)

    def abs(self):
        return math.sqrt(self.x * self.x + self.y*self.y + self.z*self.z)

    def distanceTo(self, p):
        delta = self - p
        return delta.abs()

    def normalize(self):
        return self.clone() / self.abs() 

    def rotateZ(self, angle):
        return Point(self.x * math.cos(angle) - self.y * math.sin(angle), self.x * math.sin(angle) + self.y * math.cos(angle), self.z)