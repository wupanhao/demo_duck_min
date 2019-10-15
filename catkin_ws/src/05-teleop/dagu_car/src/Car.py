from lepi import Lepi
import time

class CarDriver:
	def __init__(self):
		self.left = Lepi.MOTOR_5
		self.right = Lepi.MOTOR_2
		Lepi.motor_enable(self.left)
		Lepi.motor_enable(self.right)

	def setWheelsSpeed(self,left=0,right=0):
		Lepi.motor_set_speed(self.left,int(-left*25000))
		Lepi.motor_set_speed(self.right,int(-right*25000))

if __name__ == '__main__':
	car = CarDriver()
	car.setWheelsSpeed(0.4,0.5)
	time.sleep(1)
	car.setWheelsSpeed(0,0)
