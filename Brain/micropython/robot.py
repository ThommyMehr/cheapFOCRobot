
from machine import Timer, I2C
from imu import MPU6050
from legs import Legs
from bluefruit import BLE
from geometry import Vector, Twist, Quaternion
from time import ticks_ms, ticks_us, sleep_ms


class Robot():
    __updateFunctions = []
    selectedPID = 0  
    run = True 
    __robotParams = {
                    "Command": {"pitch" : 0.0,
                                "roll" : 0.0,
                                "hight" : 0.0,
                                "x_offset": 0.0},
                    "Set": {"pitch" : 0.0,
                                "roll" : 0.0,
                                "hight" : 0.0,
                                "x_offset": 0.0},
                    "BaseLinkFloor": Vector((0,0,0)), 
                    #"IMULink": Twist((0.05, 0.025,0.0)),
                    "BaseLink": Vector((0,0,0)),
                    "LegLink": [Vector((0.066,0, 0.044))
                            ,Vector((0.066,0,-0.044))
                            ,Vector((-0.066,0,-0.044))
                            ,Vector((-0.066,0,0.044))],
                    "FootPos":[Vector((0.044,-0.066,-0.035))
                            ,Vector((-0.044,-0.066,-0.035))
                            ,Vector((-0.044,0.066,-0.035))
                            ,Vector((0.044,0.066,-0.035))],
                    "FootGoal":[Vector((0.046,0, 0.044))
                            ,Vector((-0.086,0, 0.044))
                            ,Vector((-0.086,0, 0.044))
                            ,Vector((0.046,0, 0.044))]}
    __update_counter = 0
    torque = 0
    __up_to_date = False
    t1 = 0
    t2 = 0
    tc = 0
    linear_speed = 0.0005
    rotational_speed = 0.005
    def __init__(self,):    
        i2c = I2C(0)     
        self.ble = BLE("TommysDog")
        self.bleSettings = [("Zeroing",self.__legs_zero),
                            ("P",self.__setP),
                            ( "I",self.__setI),
                            ( "D",self.__setD),
                            ( "torque",self.__setTorque),
                            ("lin_speed",self.__setLinSpeed),
                            ("rot_speed",self.__setRotSpeed),
                            ( "shutdown",self.__shutdown)]
        self.__legs = Legs(i2c, [0x4,0x5,0x6,0x7], [False,True,True,False], [(1,1),(1,-1),(1,-1),(1,-1)], self.__robotParams["LegLink"])
        self.__imu = MPU6050(i2c)
        self.__tim1 = Timer(1)
        self.imuPID = PID(p = 0.2,i=0.02, d = 0.015)
        self.__start_i2c_deamon()    
        self.__bleSetup()    
        self.__legs_zero()   

    
    def __str__(self):
        return self.__robotParams
    
    def __repr__(self):
        return self.__robotParams

    
    def __legs_zero(self, arrow=None):
        retval = self.__legs.zero()
        self.__get_foot_vec()
        self.__robotParams["FootGoal"] = self.__robotParams["FootPos"]
        return retval
    
    def __setP(self,arrow):
        self.imuPID.kp += 0.01 * arrow
        return f"P: {self.imuPID.kp}"
    
    def __setI(self,arrow):
        self.imuPID.ki += 0.0005 * arrow
        return f"I: {self.imuPID.ki}"
    
    def __setD(self,arrow):
        self.imuPID.kd += 0.0005 * arrow
        return f"D: {self.imuPID.kd}"
    
    def __setTorque(self,arrow):
        self.torque += 1 * arrow
        return f"Torque: {self.torque}"
    
    def __setLinSpeed(self,arrow):
        self.linear_speed += 0.0001 * arrow
        return f"LinSpeed: {self.linear_speed}"
    
    def __setRotSpeed(self,arrow):
        self.rotational_speed += 0.001 * arrow
        return f"RotSpeed: {self.rotational_speed}"
    
    def __shutdown(self,arrow):
        self.torque = 0
        self.__set_next_torque((0,0))
        self.__update()
        sleep_ms(500)
        self.stop()
        self.run = False
        return f"Robot has been stopped"
        
        
        
    
    def __bleSetup(self):
        self.ble.addStates(["change hight and speed", "roll and pitch", "walk", "settings"])
        self.ble.addFunction(self.increaseSetting, "settings", "^", on="push")
        self.ble.addFunction(self.decreaseSetting, "settings", "v", on="push")
        self.ble.addFunction(self.nextSetting, "settings", ">", on="push")
        self.ble.addFunction(self.prevSetting, "settings", "<", on="push")
        self.ble.addFunction(self.moveForward, "change hight and speed", "^", on="push")
        self.ble.addFunction(self.moveForward_, "change hight and speed", "^", on="lift")
        self.ble.addFunction(self.moveBackward, "change hight and speed", "v", on="push")
        self.ble.addFunction(self.moveBackward_, "change hight and speed", "v", on="lift")
        self.ble.addFunction(self.higher, "change hight and speed", ">", on="push")
        self.ble.addFunction(self.higher_, "change hight and speed", ">", on="lift")
        self.ble.addFunction(self.lower, "change hight and speed", "<", on="push")
        self.ble.addFunction(self.lower_, "change hight and speed", "<", on="lift")
        self.ble.addFunction(self.pitchDown, "roll and pitch", "^", on="push")
        self.ble.addFunction(self.pitchDown_, "roll and pitch", "^", on="lift")
        self.ble.addFunction(self.pitchUp, "roll and pitch", "v", on="push")
        self.ble.addFunction(self.pitchUp_, "roll and pitch", "v", on="lift")
        self.ble.addFunction(self.rollRight, "roll and pitch", ">", on="push")
        self.ble.addFunction(self.rollRight_, "roll and pitch", ">", on="lift")
        self.ble.addFunction(self.rollLeft, "roll and pitch", "<", on="push")
        self.ble.addFunction(self.rollLeft_, "roll and pitch", "<", on="lift")
    


    
    def increaseSetting(self, arrow = 1):
        return self.bleSettings[self.selectedPID][1](arrow)

    def decreaseSetting(self,arrow = -1):
        return self.increaseSetting(arrow = arrow)

    def nextSetting(self, selection = 1):
        self.selectedPID = (self.selectedPID + selection)%len(self.bleSettings)
        return f"Selected: {self.bleSettings[self.selectedPID][0]}"

    def prevSetting(self):
        return self.nextSetting(selection = -1)

    def higher(self):
        self.__robotParams["Set"]["hight"] = self.linear_speed

    def higher_(self):
        self.__robotParams["Set"]["hight"] =0
        retVal = self.__robotParams["Command"]["hight"]
        return f"higher{retVal}"
        

    def lower(self):
        self.__robotParams["Set"]["hight"] = -self.linear_speed

    def lower_(self):
        self.__robotParams["Set"]["hight"] =0
        retVal = self.__robotParams["Command"]["hight"]
        return f"higher{retVal}"

    def pitchUp(self):
        self.__robotParams["Set"]["pitch"] = self.rotational_speed

    def pitchUp_(self):
        self.__robotParams["Set"]["pitch"] =0
        retVal = self.__robotParams["Command"]["pitch"]
        return f"higher{retVal}"

    def pitchDown(self):
        self.__robotParams["Set"]["pitch"] = -self.rotational_speed

    def pitchDown_(self):
        self.__robotParams["Set"]["pitch"] =0
        retVal = self.__robotParams["Command"]["pitch"]
        return f"higher{retVal}"

    def rollLeft(self):
        self.__robotParams["Set"]["roll"] = self.rotational_speed

    def rollLeft_(self):
        self.__robotParams["Set"]["roll"] =0
        retVal = self.__robotParams["Command"]["roll"]
        return f"higher{retVal}"

    def rollRight(self):
        self.__robotParams["Set"]["roll"] = -self.rotational_speed

    def rollRight_(self):
        self.__robotParams["Set"]["roll"] =0
        retVal = self.__robotParams["Command"]["roll"]
        return f"higher{retVal}"

    def moveForward(self):
        self.__robotParams["Set"]["x_offset"] = self.linear_speed

    def moveForward_(self):
        self.__robotParams["Set"]["x_offset"] =0
        retVal = self.__robotParams["Command"]["x_offset"]
        return f"higher{retVal}"

    def moveBackward(self):
        self.__robotParams["Set"]["x_offset"] = -self.linear_speed

    def moveBackward_(self):
        self.__robotParams["Set"]["x_offset"] =0
        retVal = self.__robotParams["Command"]["x_offset"]
        return f"higher{retVal}"
        
    def __contain(self, val, lim):
        return min(max(val, -1*lim), lim)
    
    def set_params(self):
        self.__robotParams["Command"]["hight"] += self.__robotParams["Set"]["hight"]
        self.__robotParams["Command"]["roll"] += self.__robotParams["Set"]["roll"]
        self.__robotParams["Command"]["pitch"] += self.__robotParams["Set"]["pitch"]
        self.__robotParams["Command"]["x_offset"] += self.__robotParams["Set"]["x_offset"]

    def param_update(self):
        CG_pos = Vector(vec = (self.__robotParams["Command"]["x_offset"], self.__robotParams["Command"]["hight"],0))
        CG_rot = Quaternion(euler=(self.__robotParams["Command"]["roll"],self.__robotParams["Command"]["pitch"], 0))
        self.__robotParams["BaseLinkFloor"] = self.__robotParams["BaseLink"] + CG_pos
        for i, link in enumerate(self.__robotParams["LegLink"]):
            self.__robotParams["FootGoal"][i] = self.__robotParams["BaseLinkFloor"] + link.get_rotated(CG_rot)
        
 

    def __start_i2c_deamon(self):
        self.__updateFunctions = self.__legs.get_update_functions()
        self.__tim1.init(period=30, mode=Timer.PERIODIC, callback=self.__run_update)
        
    def stop(self):
        self.__tim1.deinit()
    
    def __run_update(self, t):
        #self.__update_counter += 1    
        #if self.__update_counter%1 == 0:
        #self.__imu.update()
        
       # if self.__update_counter%1 == 0:
        self.set_params()
        self.param_update()
        self.__set_next_torque((self.torque,self.torque))
        self.__set_foot_vec()
        self.__up_to_date = False
        for i,function in enumerate(self.__updateFunctions):
            function()
       # self.t2 += ticks_us()-self.t1
       # if self.__update_counter%20 == 0:
       #     print(f"{self.t2/20000.}ms")
       #     self.t2 = 0    

    def __update(self):
        if self.__up_to_date:
            return
        self.__up_to_date = True
        self.__legs.update()
        
    def __get_foot_vec(self):
        self.__update()
        self.__robotParams["FootPos"]=self.__legs.get_current_vector()
        
    def __set_foot_vec(self):
        self.__legs.set_next_vector(self.__robotParams["FootGoal"])
        self.__update()

    def __set_next_torque(self, torque, index = None, relative = False):
        self.__legs.set_next_torque(torque, index, relative)


class PID():
    def __init__(self, p = 0, i = 0, d = 0):
        self.kp = p
        self.ki = i
        self.kd = d
        self.op = 0
        self.oi = 0
        self.od = 0
        self.le = 0

    def __call__(self, error):
        self.op  = self.kp * error
        self.oi += self.ki * error
        self.od += self.kd * (error - self.le)
        return self.op + self.oi + self.od









