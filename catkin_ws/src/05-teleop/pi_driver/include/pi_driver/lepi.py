import spidev
import time
import ctypes

LP_SPI = spidev.SpiDev()
LP_SPI.open(0, 1)
LP_SPI.max_speed_hz = 500000
LP_SPI.mode = 0b00
LP_SPI.bits_per_word = 8

NO_ADDR = 0

# R/W   A6 A5 A4 A3	A2 A1 A0
class Command(object):
	# R/W Command Bit
	WRITE = 0x00 
	READ = 0x01 << 7

class Group(object):
	# A6 A5 A4 A3 , Group Register Bit
	# 0 for System,1-5 for Motor,6-10 for Sensor
	SYSTEM = 0x00 << 3
	
	MOTOR_1 = 0x01 << 3
	MOTOR_2 = 0x02 << 3
	MOTOR_3 = 0x03 << 3
	MOTOR_4 = 0x04 << 3
	MOTOR_5 = 0x05 << 3

	SENSOR_1 = 0x06 << 3
	SENSOR_2 = 0x07 << 3
	SENSOR_3 = 0x08 << 3
	SENSOR_4 = 0x09 << 3
	SENSOR_5 = 0x0a << 3

class Motor(object):
	# A2 A1 A0 , Register Bit in Group
	TYPE = 0 # 0 for EV3 Motor
	ENABLE = 1 # 1 for enable, 0 for disable
	STATE = 2 
	POSITION = 3 # Current Position
	SETPOSITION = 4 # Target Position
	SPEED = 5 # from -65535 to 65535 , 0 means brake

class Message(object):
	@staticmethod
	def GetMotorState(port):
		return Command.READ | port | Motor.STATE
	@staticmethod
	def GetCurrentPosition(port):
		return Command.READ | port | Motor.POSITION
	@staticmethod
	def SetCurrentPosition(port):
		return Command.WRITE | port | Motor.POSITION
	@staticmethod
	def GetTargetPosition(port):
		return Command.READ | port | Motor.SETPOSITION
	@staticmethod
	def SetTargetPosition(port):
		return Command.WRITE | port | Motor.SETPOSITION
	@staticmethod
	def GetMortorSpeed(port):
		return Command.READ | port | Motor.SPEED
	@staticmethod
	def SetMortorSpeed(port):
		return Command.WRITE | port | Motor.SPEED
	@staticmethod
	def SetMortorEnable(port):
		return Command.WRITE | port | Motor.ENABLE
class Lepi(object):
	SYSTEM = 0x00 << 3
	MOTOR_1 = 1
	MOTOR_2 = 2
	MOTOR_3 = 3
	MOTOR_4 = 4
	MOTOR_5 = 5
	SENSOR_1 = 1
	SENSOR_2 = 2
	SENSOR_3 = 3
	SENSOR_4 = 4
	SENSOR_5 = 5
	Motors = {1:Group.MOTOR_1,2:Group.MOTOR_2,3:Group.MOTOR_3,4:Group.MOTOR_4,5:Group.MOTOR_5}
	Sensors = {1:Group.SENSOR_1,2:Group.SENSOR_2,3:Group.SENSOR_3,4:Group.SENSOR_4,5:Group.SENSOR_5}
	spi = LP_SPI
	@classmethod
	def spi_read_32(self, MessageType):
		"""
		Read a 32-bit value over SPI

		Keyword arguments:
		MessageType -- the SPI message type

		Returns :
		value
		"""
		outArray = [MessageType, 0, 0, 0, 0, 0]
		reply = self.spi.xfer2(outArray)
		# print(reply[2:6])
		cint32 = ctypes.c_int32( (reply[5] << 24) | (reply[4] << 16) | (reply[3] << 8) | reply[2] )
		return cint32.value
		# value = int((reply[5] << 24) | (reply[4] << 16) | (reply[3] << 8) | reply[2])
		# print(value)
		# if value >> 31 == 1:
		# 	value = value - 0x100000000
		# return value

	@classmethod
	def spi_write_32(self, MessageType, Value):
		"""
		Send a 32-bit value over SPI

		Keyword arguments:
		MessageType -- the SPI message type
		Value -- the value to be sent
		"""
		outArray = [MessageType, (Value & 0xFF), ((Value >> 8) & 0xFF), ((Value >> 16) & 0xFF), ((Value >> 24) & 0xFF)]
		# print(outArray[1:5])
		self.spi.xfer2(outArray)
	@classmethod
	def motor_enable(self,port):
		if self.Motors.has_key(port):
			self.spi_write_32(Message.SetMortorEnable(self.Motors[port]),1)
		else:
			print('wrong motor number')
	@classmethod
	def motor_disable(self,port):
		if self.Motors.has_key(port):
			self.spi_write_32(Message.SetMortorEnable(self.Motors[port]),0)
		else:
			print('wrong motor number')
	@classmethod
	def motor_set_speed(self,port,speed):
		if self.Motors.has_key(port):
			self.spi_write_32(Message.SetMortorSpeed(self.Motors[port]),speed)
		else:
			print('wrong motor number')
	@classmethod
	def motor_get_current_position(self,port):
		if self.Motors.has_key(port):
			return self.spi_read_32(Message.GetCurrentPosition(self.Motors[port]))
		else:
			print('wrong motor number')
	@classmethod
	def motor_set_target_position(self,port,position):
		if self.Motors.has_key(port):
			self.spi_write_32(Message.SetTargetPosition(self.Motors[port]),position)
		else:
			print('wrong motor number')
	@classmethod
	def motor_get_speed(self,port):
		if self.Motors.has_key(port):
			return self.spi_read_32(Message.GetMortorSpeed(self.Motors[port]))
		else:
			print('wrong motor number')

# print(Command.WRITE | Lepi.MOTOR_3 | Motor.SPEED)

if __name__ == '__main__':
	# import time
	# Lepi.motor_set_speed(Lepi.MOTOR_2,0)
	# time.sleep(0.02)
	# Lepi.motor_enable(Lepi.MOTOR_2)
	# time.sleep(0.02)
	# print(Lepi.motor_get_current_position(Lepi.MOTOR_2))
	# time.sleep(0.02)
	# print(Lepi.motor_set_target_position(Lepi.MOTOR_2,720))
	# time.sleep(0.02)

	# # Lepi.motor_set_speed(Lepi.MOTOR_2,35000)

	# time.sleep(2)
	# print(Lepi.motor_get_current_position(Lepi.MOTOR_2))
	# time.sleep(0.02)

	Lepi.motor_enable(Lepi.MOTOR_2)
	time.sleep(0.02)
	Lepi.motor_set_speed(Lepi.MOTOR_2,200)
	time.sleep(0.02)	
	print(Lepi.motor_get_speed(Lepi.MOTOR_2))
	time.sleep(0.02)
	Lepi.motor_set_speed(Lepi.MOTOR_2,20000)
	time.sleep(0.02)
	print(Lepi.motor_get_speed(Lepi.MOTOR_2))
	Lepi.motor_disable(Lepi.MOTOR_2)
	time.sleep(0.02)	
