#!coding:utf-8
import time
import smbus
import RPi.GPIO as GPIO

ReadButtonState =0x82

ReadGSensor  =0x83
ReadGyroSensor  =0x84
ReadMagnetometer = 0x85

Button = {
    0x81:'btn1down',
    0x82:'btn2down',
    0x83:'btn3down',
    0x84:'btn4down',
    0x85:'btn5down',
    0x86:'btn6down',
    0x87:'btn7down',
    0x88:'btn8down',
    0x89:'btn9down',

    0x01:'btn1up',
    0x02:'btn2up',
    0x03:'btn3up',
    0x04:'btn4up',
    0x05:'btn5up',
    0x06:'btn6up',
    0x07:'btn7up',
    0x08:'btn8up',
    0x09:'btn9up',

    0x99:'shutDownRequest',
    0x4A:'chargeIn',
    0x40:'chargeOut',
}

ButtonMap = {
    # button down
    0x81:67, # Choose 'C' 67
    0x82:38, #  'ArrowUp': 38
    0x83:66, # Back 'B' 66
    0x84:37, # 'ArrowLeft': 37
    0x85:13, # 'Enter': 13
    0x86:39, # 'ArrowRight': 39
    0x87:82, # Run 'R' 82
    0x88:40, # 'ArrowDown': 40
    0x89:27, # 'Esc' 27

    # button up
    0x01:67,
    0x02:38,
    0x03:66,
    0x04:37,
    0x05:13,
    0x06:39,
    0x07:82,
    0x08:40,
    0x09:27,

    0x99:'shutDownRequest',
    0x4A:'chargeIn',
    0x40:'chargeOut',
}

class I2cDriver:
    def __init__(self,btn_handler=None):
        self.m031_addr = 0x15
        self.int_pin = 22
        self.bus =smbus.SMBus(1)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.int_pin,GPIO.IN)
        GPIO.add_event_detect(self.int_pin,GPIO.BOTH,callback=self.int_handler,bouncetime=20)
        self.btn_handler = btn_handler
    def enable_sensor(self,sensor,speed):
        self.bus.write_byte_data(self.m031_addr,0x46,0x47)
    def read_sensor_value(self,sensor):
        return bus.read_i2c_block_data(self.m031_addr,sensor,6)
    def read_power_value(self):
        return bus.read_i2c_block_data(self.m031_addr,0x8A,2)
    def read_power_state(self,key):
        return bus.read_i2c_block_data(self.m031_addr,key,2)
    def int_handler(self,channel):
        btn=self.bus.read_byte_data(self.m031_addr,ReadButtonState)
        if self.btn_handler!=None:
            if self.btn_handler(btn):
                return
        print(btn)
        if Button.has_key(btn):
            print(Button[btn])

if __name__ == '__main__':
    def test_print(data):
        print('read value %d ' % (data))
        return False
    driver = I2cDriver(test_print)
    while True:
        time.sleep(1)

# while(1):
#     print(bus.read_i2c_block_data(adress,0x83,6))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x84,6))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x85,6))
#     time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x8A,2))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x8B,4))
#     print(bus.read_byte_data(adress,0x8C))
