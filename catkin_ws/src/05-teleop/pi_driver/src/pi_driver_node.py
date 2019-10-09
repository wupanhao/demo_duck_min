#!/usr/bin/python
from pi_driver import Lepi
from pi_driver.msg import MotorSetSpeed,MotorSetPosition
from pi_driver.srv import MotorGetPosition,MotorGetPositionResponse

import rospkg
import rospy
from std_msgs.msg import UInt8,Int32

class PiDriverNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		self.sub_motor_enable = rospy.Subscriber("~motor_enable", UInt8 , self.cbMotorEnable, queue_size=1)
		self.sub_motor_disable =rospy.Subscriber("~motor_disable", UInt8 , self.cbMotorDisable, queue_size=1)
		self.sub_motor_set_speed =rospy.Subscriber("~motor_set_speed", MotorSetSpeed , self.cbMotorSetSpeed, queue_size=1)
		self.sub_motor_set_position = rospy.Subscriber('~motor_set_position', MotorSetPosition, self.cbMotorSetPosition)
		self.srv_motor_get_position = rospy.Service('~motor_get_position', MotorGetPosition, self.cbMotorGetPosition)

		rospy.loginfo("[%s] Initialized......" % (self.node_name))

	def cbMotorEnable(self,msg):
		# print(msg)
		Lepi.motor_enable(msg.data)
	def cbMotorDisable(self,msg):
		Lepi.motor_disable(msg.data)
		# print(msg)
	def cbMotorSetSpeed(self,msg):
		# print(msg)
		Lepi.motor_set_speed(msg.port,msg.speed)
	def cbMotorSetPosition(self,msg):
		print(msg)
		Lepi.motor_set_target_position(msg.port,msg.position)
	def cbMotorGetPosition(self,msg):
		print(msg)
		position = Lepi.motor_get_current_position(msg.port)
		return MotorGetPositionResponse(position)

if __name__ == '__main__':
	rospy.init_node('pi_driver_node', anonymous=False)
	node = PiDriverNode()
	# rospy.on_shutdown(node.onShutdown)
	# thread.start_new_thread(camera_node.startCaptureRawCV, ())
	rospy.spin()
