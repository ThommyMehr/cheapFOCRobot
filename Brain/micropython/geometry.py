import math


class Vector():

    vec = [0,0,0]

    def __init__(self, vec = None, x = None, y = None, z = None):
        if vec is not None:
            x,y,z= vec
        self.vec = [x,y,z]

    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self, x):
        self[0] = x

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self, y):
        self[1] = y

    @property
    def z(self):
        return self[2]

    @z.setter
    def z(self, z):
        self[2] = z

    def angle(self, other):
        theta = math.acos(self.dot(other)/(abs(self)*abs(other)))
        return theta

    def __getitem__(self, key):
        return self.vec[key]

    def __setitem__(self, key, value):
        self.vec[key] = value

    def __add__(self, other):
        if isinstance(other, Vector):
            return Vector([self[i]+other[i] for i in range(3)])
    
    def __sub__(self, other):
        if isinstance(other, Vector):
            return Vector([self[i]-other[i] for i in range(3)])
    
    def __mul__(self, other):
        if isinstance(other, Vector):
            return
        else:
            return Vector([self[i]*other for i in range(3)])

    def __truediv__(self, other):
        if isinstance(other, Vector):
            return
        else:
            return Vector([self[i]/other for i in range(3)])

    def dot(self, other):
        s = 0
        for i in range(3):
            s += self[i] * other[i]
        return s

    def dist(self, other):
        temp = self - other
        return abs(temp)
    
    def __abs__(self):
        s = 0
        for i in range(3):
            s += self[i]**2
        return math.sqrt(s)
    
    def __len__(self):
        return 3

    def __lt__(self, other):
        return abs(self)<abs(other)
    def __le__(self, other):
        return abs(self)<=abs(other)
    def __gt__(self, other):
        return abs(self)>abs(other)
    def __ge__(self, other):
        return abs(self)>=abs(other)
    def __eq__(self, other):
        return abs(self)==abs(other)
    def __ne__(self, other):
        return abs(self)!=abs(other)

    def normalize(self):
        m = abs(self)
        self.x = self.x / m
        self.y = self.y / m
        self.z = self.z / m
    
    def get_normalized(self):
        result = Vector(x = self.x, y = self.y, z = self.z)
        result.normalize()
        return result

    def rotate(self, a_quat):
        p = Quaternion(w = 0.0, x = self.x, y = self.y, z = self.z)
        p = a_quat * p
        p = p * a_quat.get_conjugate()
        # By magic quaternion p is now [0, x', y', z']
        self.x = p.x
        self.y = p.y
        self.z = p.z
    
    def get_rotated(self, a_quat):
        r = Vector(x = self.x, y = self.y, z = self.z)
        r.rotate(a_quat)
        return r

    def cross(self, other):
        x_1 = self[1] * other[2] - self[2] * other[1]
        x_2 = self[2] * other[0] - self[0] * other[2]
        x_3 = self[0] * other[1] - self[1] * other[0]
        return Vector((x_1,x_2,x_3))

    def __str__(self):
        return f" x: {self[0]}\n y: {self[1]}\n z: {self[2]}"
    
    def __repr__(self):
        return f" x: {self[0]}\n y: {self[1]}\n z: {self[2]}"


class Quaternion:

    quat = [1,0,0,0]

    def __init__(self, euler = None, quat = None, w=None, x=None, y=None, z=None):
        if euler is not None:
            self.__euler_to_quaternion(euler)
        elif quat is not None:
            self.quat = quat
        else:
            self.quat = [w, x,y,z]

    def __getitem__(self, key):
        return self.quat[key]

    def __setitem__(self, key, value):
        self.quat[key] = value

    @property
    def w(self):
        return self[0]

    @w.setter
    def w(self, w):
        self[0] = w

    @property
    def x(self):
        return self[1]

    @x.setter
    def x(self, x):
        self[1] = x

    @property
    def y(self):
        return self[2]

    @y.setter
    def y(self, y):
        self[2] = y

    @property
    def z(self):
        return self[3]

    @z.setter
    def z(self, z):
        self[3] = z
    
    def __euler_to_quaternion(self, euler):
        roll, pitch, yaw = euler
        self.x = math.sin(roll/2) * math.cos(yaw/2) * math.cos(pitch/2) - math.cos(roll/2) * math.sin(yaw/2) * math.sin(pitch/2)
        self.y = math.cos(roll/2) * math.sin(yaw/2) * math.cos(pitch/2) + math.sin(roll/2) * math.cos(yaw/2) * math.sin(pitch/2)
        self.z = math.cos(roll/2) * math.cos(yaw/2) * math.sin(pitch/2) - math.sin(roll/2) * math.sin(yaw/2) * math.cos(pitch/2)
        self.w = math.cos(roll/2) * math.cos(yaw/2) * math.cos(pitch/2) + math.sin(roll/2) * math.sin(yaw/2) * math.sin(pitch/2)
    
    def to_euler(self):
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(t3, t4)
        return (roll, pitch, yaw)

    def __mul__(self, a_quat):
        result = Quaternion(
            w = self.w * a_quat.w - self.x * a_quat.x -
            self.y * a_quat.y - self.z * a_quat.z,

            x = self.w * a_quat.x + self.x * a_quat.w +
            self.y * a_quat.z - self.z * a_quat.y,

            y = self.w * a_quat.y - self.x * a_quat.z +
            self.y * a_quat.w + self.z * a_quat.x,

            z = self.w * a_quat.z + self.x * a_quat.y -
            self.y * a_quat.x + self.z * a_quat.w)
        return result

    def get_conjugate(self):
        result = Quaternion(w =self.w, x = -self.x,y= -self.y,z= -self.z)
        return result

    def __abs__(self):
        return math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y +
                    self.z * self.z)

    def normalize(self):
        m = abs(self)
        self.w = self.w / m
        self.x = self.x / m
        self.y = self.y / m
        self.z = self.z / m

    def get_normalized(self):
        result = Quaternion(w = self.w, x = self.x, y = self.y, z = self.z)
        result.normalize()
        return result


    def __str__(self):
        return f" w: {self.w}\n x: {self.x}\n y: {self.y}\n z: {self.z}"
    
    def __repr__(self):
        return f" w: {self.w}\n x: {self.x}\n y: {self.y}\n z: {self.z}"



class Twist():
    def __init__(self,vec, quat= None, euler = None):
        self.vec = Vector(vec= vec)
        if quat is not None:
            self.quat = Quaternion(quat=quat)
        elif euler is not None:
            self.quat = Quaternion(euler=euler)
        else:
            self.quat = Quaternion(quat=[1,0,0,0])

    def __str__(self):
        return f"Vector:\n{self.vec}\nQuaternion:\n{self.quat}"
    
    def __repr__(self):
        return f"Vector:\n{self.vec}\nQuaternion:\n{self.quat}"
    
