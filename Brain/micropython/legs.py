from kinematicLUT import legLUT
from struct import unpack, pack
from geometry import Vector
from time import sleep_us
class BGC():

    __state_updated = False
    def __init__(self, i2c, addr, lUS = False, sign = (1,1), deadband = 2, gain = 2, approachSpeed = 0.95, offset = (0,-2475)):
        self.__i2c = i2c
        self.addr = addr
        self.__sign = sign
        self.__offset = offset
        self.__old_val = 1 - approachSpeed
        self.__new_val = approachSpeed
        self.__state = bytearray(6)
        self.__order = bytearray(6)
        self.__motor_set = [0,1]
        self.__current_position = [0,0]
        self.__current_torque = [0,0]
        self.__next_position = [0,0]
        self.__order_position = [0,0]
        self.__order_torque = [0,0]
        self.__state_updated = False
        self.__last_update = 40
        if lUS:
            self.__motor_set = [1,0]
        self.__deadband(deadband)
        self.__gain(gain)
        
    
    def torque(self, num = None):
        self.__decode_state()
        if num is None:
            return self.__current_torque
        return self.__current_torque[num]

    def angle(self, num = None):
        self.__decode_state()
        if num is None:
            return self.__current_position
        return self.__current_position[num]

    def maxTorque(self, vals):
        for index in [0,1]:
            self.__order_torque[index] = vals[index]
        self.__last_update = 80

    def _newAngle(self, vals):
        for index in [0,1]:
            self.__next_position[index] = vals[index]
        self.__last_update = 80
        
    def getNewAngle(self):
        return self.__next_position
    
    def newAngle(self, vals):
        for index in [0,1]:
            self.__next_position[index] = vals[index]
            self.__order_position[index]= vals[index]
        self.__last_update = 80
    
    def update(self):
        self.__get_state()
        self.__encode_order()
        self.__set_order()
        

    def __get_state(self):
        self.__state = self.__i2c.readfrom(self.addr,6)
        self.__state_updated = False
    
    def __decode_state(self):
        if self.__state_updated:
            return
        buf = unpack('BBhh', self.__state)
        self.__current_position =  [buf[2+self.__motor_set[0]] * self.__sign[self.__motor_set[0]] - self.__offset[0], 
                                    buf[2+self.__motor_set[1]] * self.__sign[self.__motor_set[1]] - self.__offset[1]]
        self.__current_torque = [buf[self.__motor_set[0]], buf[self.__motor_set[1]]]
        self.__state_updated = True

    def __encode_order(self):
        for index in [0,1]:
            self.__order_position[index] = int(self.__old_val* self.__order_position[index] + self.__new_val* self.__next_position[index])
        self.__order = pack('BBhh',
                            self.__order_torque[self.__motor_set[0]],
                            self.__order_torque[self.__motor_set[1]],
                            (self.__order_position[self.__motor_set[0]] + self.__offset[self.__motor_set[0]]) * self.__sign[0], 
                            (self.__order_position[self.__motor_set[1]] + self.__offset[self.__motor_set[1]]) * self.__sign[1])
    
    def __disable(self):
        self.__order = pack('bbhh', 0,0,0,0)

    def __set_order(self):
        if self.__last_update <= 0:
            self.__disable()
        elif self.__last_update > 0:
            self.__last_update -= 1
        self.__i2c.writeto(self.addr, self.__order)
        

    def zero(self):
        orderBuffer = pack('bbhh',0xFF,0,0, 0)
        self.__i2c.writeto(self.addr, orderBuffer)
        
    def __deadband(self, deadband):
        orderBuffer = pack('bbhh',0xFE,deadband,0, 0)
        self.__i2c.writeto(self.addr, orderBuffer)
        
    def __gain(self, gain):
        orderBuffer = pack('bbhh',0xFD,gain,gain, 0)
        self.__i2c.writeto(self.addr, orderBuffer)

    


class Legs():
    
    def __init__(self, i2c, adresses, lUSs, signs, links):
        self.__BGCs = [BGC(i2c,addr, lUS=lUS , sign=sign) for addr, lUS, sign in zip(adresses,lUSs,signs) ]
        self.__legs = [{'current': {'torque': [0, 0], 'position': [0, 0]}, 'next': {'torque': [0, 0], 'position': [0, 0]}}, {'current': {'torque': [0, 0], 'position': [0, 0]}, 'next': {'torque': [0, 0], 'position': [0, 0]}}, {'current': {'torque': [0, 0], 'position': [0, 0]}, 'next': {'torque': [0, 0], 'position': [0, 0]}}, {'current': {'torque': [0, 0], 'position': [0, 0]}, 'next': {'torque': [0, 0], 'position': [0, 0]}}]
        self.__legs = list(zip(self.__BGCs, self.__legs))
        self.__links = links
        self.__kinematic = legLUT()


    def go_to_zero(self, torque = (5,5)):
        for leg, _ in self.__legs:
            a, b = leg.angle()
            leg.maxTorque(torque)
            leg.forceNewAngle([a-200,b+200])

    def zero(self):
        for leg, _ in self.__legs:
            leg.zero()        
        return "All Legs Zeroed"


    def update(self,):
        for bgc, leg in self.__legs:
            leg["current"]["torque"] = bgc.torque()
            leg["current"]["position"] = self.__kinematic.forward(bgc.angle())
            bgc.maxTorque(leg["next"]["torque"])
            bgc.newAngle(self.__kinematic.inverse(leg["next"]["position"]))

    def set_next_vector(self, new_vec, index=None):
        if index is None:
            if isinstance(new_vec, Vector):
                new_vec = [new_vec for _ in range(4)]
            ret_arr = []
            for vec, link in zip(new_vec, self.__links):
                vec = link - vec
                ret_arr.append((vec[0], vec[1]))
            self.set_next_position(ret_arr)
        else:
            new_vec = self.__links[index] - new_vec
            self.set_next_position((new_vec[0], new_vec[1]), index)

    def set_next_position(self, position, index = None, relative = False):
        if index is None:
            if isinstance(position,Vector):
                position = [position for _ in range(4)]
            for pos, leg in zip(position,self.__legs):
                _,leg = leg
                if relative:
                    current_x, current_y = leg["current"]["position"]
                else:
                    current_x, current_y = [0,0]
                x, y = pos
                leg["next"]["position"] = [current_x+x, current_y+y]
        else:
            if relative:
                current_x, current_y = self.__legs[index][1]["current"]["position"]
            else:
                current_x, current_y = [0,0]
            x, y = position
            self.__legs[index][1]["next"]["position"] = [current_x+x, current_y+y]

    def set_next_torque(self, torque, index = None, relative = False):
        if index is None:
            for _, leg in self.__legs:
                if relative:
                    current_t1, current_t2 = leg["current"]["torque"]
                else:
                    current_t1, current_t2 = [0,0]
                t1, t2 = torque
                leg["next"]["torque"] = [current_t1+t1, current_t2+t2]
        else:
            if relative:
                current_t1, current_t2 = self.__legs[index][1]["current"]["torque"]
            else:
                current_t1, current_t2 = [0,0]
            t1, t2 = torque
            self.__legs[index][1]["next"]["torque"] = [current_t1+t1, current_t2+t2]

    def get_current_torque(self, index = None):
        if index is not None:
            return self.__legs[index][1]["current"]["torque"]
        else:
            return [leg["current"]["torque"] for _, leg in self.__legs]

    def get_current_position(self, index = None):
        if index is not None:
            return self.__legs[index][1]["current"]["position"]
        else:
            return [leg["current"]["position"] for _, leg in self.__legs]

    def get_current_vector(self,index = None):
        val = self.get_current_position(index)
        if index is None:
            ret_arr = []
            for value, link in zip(val, self.__links):
                temp = Vector((value[0],value[1], 0))
                
                ret_arr.append(link + temp)
            return ret_arr
        else:
            vec = self.__links[index]
            val = Vector((value[0],value[1], 0))
            return vec+val

    def get_update_functions(self):
        funcs = []
        for leg,_ in self.__legs:
            funcs.append(leg.update)
        return funcs







