import pigpio
import time

class Lepi:
	gpio = pigpio.pi()
	@classmethod
	def motor_set_pwm(self,motor_pwm_pin,speed):
		if motor_pwm_pin != 12 and motor_pwm_pin != 13:
			print('wrong pwm pin')
			return
		self.gpio.hardware_PWM(motor_pwm_pin,100,speed) # 100hz , (speed/100 0000.0) % duty
	@classmethod
	def motor_init(self,pwm_pin,dir1,dir2):
		self.gpio.set_mode(pwm_pin,pigpio.ALT5)
		self.gpio.set_mode(dir1,pigpio.OUTPUT)
		self.gpio.set_mode(dir2,pigpio.OUTPUT)
	@classmethod
	def motor_stop(self,pwm_pin,dir1,dir2):
		self.gpio.write(pwm_pin,0)
		self.gpio.write(dir1,0)
		self.gpio.write(dir2,0)
	@classmethod
	def motor_set_dir(self,dir_pin,value):
		if value!=pigpio.HIGH and value!=pigpio.LOW:
			print('wrong value')
			return
		self.gpio.write(dir_pin,value)

class CarDriver:
	def __init__(self):
		self.motor_A = [12,6,17] # pwm_pin dir1 dir2
		self.motor_B = [13,24,23] # pwm_pin dir1 dir2
		self.max_speed = 100000 # 100 0000 Maximun
		Lepi.motor_init(self.motor_A[0],self.motor_A[1],self.motor_A[2])
		Lepi.motor_init(self.motor_B[0],self.motor_B[1],self.motor_B[2])

	def motor_set_speed(self,motor,speed):
		if speed == 0:
			Lepi.motor_stop(motor[0])
			return
		elif speed > 0:
			Lepi.motor_set_dir(motor[1],pigpio.HIGH)
			Lepi.motor_set_dir(motor[2],pigpio.LOW)
		elif speed < 0 :
			Lepi.motor_set_dir(motor[1],pigpio.LOW)
			Lepi.motor_set_dir(motor[2],pigpio.HIGH)
		Lepi.motor_set_pwm(motor[0],int(abs(speed)))

	def setWheelsSpeed(self,left=0,right=0):
		self.motor_set_speed(self.motor_A,int(left*self.max_speed))
		self.motor_set_speed(self.motor_B,int(right*self.max_speed))

if __name__ == '__main__':
	car = CarDriver()
	car.setWheelsSpeed(0.4,0.5)
	time.sleep(1)