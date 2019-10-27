from lepi import Lepi
import time

class CarDriver:
	def __init__(self):
		self.left = Lepi.MOTOR_5
		self.right = Lepi.MOTOR_3
		Lepi.motor_enable(self.left)
		time.sleep(0.02)
		Lepi.motor_enable(self.right)

	def setWheelsSpeed(self,left=0,right=0):
		Lepi.motor_set_speed(self.left,int(-left*58500))
		time.sleep(0.02)
		Lepi.motor_set_speed(self.right,int(-right*65000))

if __name__ == '__main__':
	car = CarDriver()
	car.setWheelsSpeed(1,1)
	time.sleep(4)
	car.setWheelsSpeed(0,0)
