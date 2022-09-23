import math

class MPU6050():
    __zero_countdown = 40
    def __init__(self, i2c, addr=0x68, scaling = (1,-1,-1), smoothing = 0.8):
        self.__iic = i2c
        self.__addr = addr
        self.__iic.writeto(self.__addr, bytearray([107, 0]))
        self.roll = 0
        self.pitch = 0
        self.raw_roll = 0
        self.raw_pitch = 0
        self.__old_val = smoothing
        self.__new_val = 1-smoothing
        self.__scaling = scaling
        self.__vals = [0,0,0,0,0,0]
        self.__offset = [0,0,0,0,0,0]
        self.__zeroed = False

    def get_raw_values(self):
        a = self.__iic.readfrom_mem(self.__addr, 0x3B, 6)
        return a

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def zero(self):
        self.get_raw()
        for i, val in enumerate(self.__vals):
            if i != 2:
                self.__offset[i] = 0.9*self.__offset[i] + 0.1*val
            
    def get_values(self):
        raw_ints = self.get_raw_values()
        self.__vals[0] = self.bytes_toint(raw_ints[0], raw_ints[1])*self.__scaling[0] -  self.__offset[0]      #X
        self.__vals[1] = self.bytes_toint(raw_ints[2], raw_ints[3])*self.__scaling[1] -  self.__offset[1]       #Y
        self.__vals[2] = self.bytes_toint(raw_ints[4], raw_ints[5])*self.__scaling[2] -  self.__offset[2]       #Z
        
    def get_raw(self):
        raw_ints = self.get_raw_values()
        self.__vals[0] = self.bytes_toint(raw_ints[0], raw_ints[1])*self.__scaling[0]       #X
        self.__vals[1] = self.bytes_toint(raw_ints[2], raw_ints[3])*self.__scaling[1]        #Y
        self.__vals[2] = self.bytes_toint(raw_ints[4], raw_ints[5])*self.__scaling[2]        #Z

    def update(self):
        if self.__zero_countdown > 0:
            self.__zero_countdown -= 1
            self.zero()
        self.__update()

    def __update(self):
        self.get_values()
        self.raw_pitch = math.atan2(self.__vals[0],self.__vals[2])
        self.raw_roll  = math.atan2(self.__vals[1],self.__vals[2])*-1
        self.pitch = self.__old_val*self.pitch + self.__new_val*self.raw_pitch
        self.roll  = self.__old_val*self.roll  + self.__new_val*self.raw_roll

