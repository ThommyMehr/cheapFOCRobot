
from time import sleep_ms
import bluetooth
from struct import unpack, pack
from machine import freq


class BLE():
    def __init__(self, name):
        print(freq())
        freq(120_000_000)
        self.name = name
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.bface = ButtenInterface()
        freq(400_000_000)
        pass#self.led = Pin(2, Pin.OUT)
        pass#self.timer1 = Timer(0)
        
        self.disconnected()
        
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        
    def addStates(self, states):
        self.bface.addStates(states)    

    def addFunction(self, func, state,button, on):
        self.bface.addFunction(func, state,button, on)

    def connected(self):        
        pass#self.timer1.deinit()

    def disconnected(self):        
        pass#self.timer1.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: self.led((1 + self.led.value())%2)) 

    def ble_irq(self, event, data):
        if event == 1:
            '''Central disconnected'''
            self.connected()
            self.led(1)
        
        elif event == 2:
            '''Central disconnected'''
            self.advertiser()
            self.disconnected()
        
        elif event == 3:
            '''New message received'''            
            buffer = self.ble.gatts_read(self.rx)
            message = unpack('BBBBB', buffer)
            send_back = self.bface((message[2]-49,message[3]-48))
            if send_back is not None:
                self.send(send_back)
       
            
           
    def register(self):        
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
            
        BLE_NUS = bluetooth.UUID(NUS_UUID)
        BLE_RX = (bluetooth.UUID(RX_UUID), bluetooth.FLAG_WRITE)
        BLE_TX = (bluetooth.UUID(TX_UUID), bluetooth.FLAG_NOTIFY)
            
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)

    def send(self, data):
        self.ble.gatts_notify(0, self.tx, data + '\n')

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        self.ble.gap_advertise(100, bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name)
        
class ButtenInterface():
    #        1,2,3,4,^,v,<,>
    __state = 0
    __buttonID = {'^':0,'v':1,'<':2,'>':3}
    __buttonState = {'push': 1, "lift": 0}
    __stateID = dict()
    __state_decriptions = []
    __functions = [[[None,None],[None,None],[None,None],[None,None]],
                    [[None,None],[None,None],[None,None],[None,None]],
                    [[None,None],[None,None],[None,None],[None,None]],
                    [[None,None],[None,None],[None,None],[None,None]],]
    def __call__(self, button):
        button, state = button
        if button > 3:
            return self.__arrowButtons(button, state)
        elif state == 0:
            return self.__numButtons(button) 

    def __numButtons(self, button):
        self.__state = button
        return self.__state_decriptions[self.__state]

    def __arrowButtons(self, button, buttonState):
        exec = self.__functions[self.__state][button-4][buttonState]
        if exec is None:
            return None
        return exec()
    
    def addStates(self, states):
        self.__state_decriptions = states
        for i, state in enumerate(states):
            self.__stateID[state] = i


    def addFunction(self, func, state, button, on="push"):
        self.__functions[self.__stateID[state]][self.__buttonID[button]][self.__buttonState[on]] = func



