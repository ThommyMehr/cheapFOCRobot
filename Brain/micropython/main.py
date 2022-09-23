from legs import Legs
from machine import Timer, I2C

i2c = I2C(0)
tim1 = Timer(1)

legs = Legs(i2c, [0x4,0x5,0x6,0x7], [False,True,True,False], [(1,-1),(1,-1),(1,-1),(1,-1)])
print("LEGS initialized")
updateFunctions = legs.get_update_functions()
counter = 0
def runUpdate(t):
    for function in updateFunctions:
        function()
        
tim1.init(period=50, mode=Timer.PERIODIC, callback=runUpdate)
print("interrupt timer started")

legs.set_next_torque((10,10))

