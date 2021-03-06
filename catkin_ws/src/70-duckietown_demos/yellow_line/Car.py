from lepi import Lepi
import time

class CarDriver:
	def __init__(self):
		self.left = Lepi.MOTOR_1
		self.right = Lepi.MOTOR_5
		Lepi.motor_set_enable(self.left,1)
		time.sleep(0.02)
		Lepi.motor_set_enable(self.right,1)

	def setWheelsSpeed(self,left=0,right=0):
		Lepi.motor_set_speed(self.left,int(left*100))
		time.sleep(0.02)
		Lepi.motor_set_speed(self.right,int(right*100))
class CarDriver2:
	def __init__(self):
		self.steer = Lepi.MOTOR_5
		self.speed = Lepi.MOTOR_1
		Lepi.motor_set_enable(self.steer,1)
		time.sleep(0.02)
		Lepi.motor_set_enable(self.speed,1)

	def setWheelsSpeed(self,left=0,right=0):
		speed = int(-(left+right)/2.0*100)
		#speed = int(-0.6*65500) # or use a constant
		steer = int((left-right)*90.0)
		Lepi.motor_set_speed(self.speed,speed)
		Lepi.servo_set_angle(self.steer,steer)
if __name__ == '__main__':
	car = CarDriver()
	car.setWheelsSpeed(0.2,0.2)
	time.sleep(2)
        car.setWheelsSpeed(0.6,0.6)
        time.sleep(2)
        car.setWheelsSpeed(1,1)
        time.sleep(2)
	car.setWheelsSpeed(0,0)
