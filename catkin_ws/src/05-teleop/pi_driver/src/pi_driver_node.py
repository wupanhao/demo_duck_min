#!/usr/bin/python
from pi_driver import Lepi,I2cDriver,ButtonMap
from pi_driver.msg import ButtonEvent
from pi_driver.srv import SetInt32,GetInt32,SetInt32Response,GetInt32Response

import rospkg
import rospy
from std_msgs.msg import UInt8,Int32

class PiDriverNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		self.srv_motor_set_enable = rospy.Service("~motor_set_enable", SetInt32 , self.srvMotorSetEnable, queue_size=1)
		self.srv_motor_get_enable = rospy.Service("~motor_get_enable", GetInt32 , self.srvMotorGetEnable, queue_size=1)
		self.srv_motor_set_speed = rospy.Service("~motor_set_speed", SetInt32 , self.srvMotorSetSpeed, queue_size=1)
		self.srv_motor_get_speed = rospy.Service("~motor_get_speed", GetInt32 , self.srvMotorGetSpeed, queue_size=1)
		self.srv_motor_set_position = rospy.Service('~motor_set_position', SetInt32, self.srvMotorSetPosition)
		self.srv_motor_get_position = rospy.Service('~motor_get_position', GetInt32, self.srvMotorGetPosition)
		self.srv_motors_get_info = rospy.Service('~motors_get_info', GetMotorsInfo, self.srvMotorsGetInfo)
		self.pub_button_event =rospy.Publisher("~button_event", ButtonEvent, queue_size=1)
		self.i2c_driver = I2cDriver(self.pubButton)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))

	def srvMotorSetEnable(self,msg):
		# print(msg)
		Lepi.motor_set_enable(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetEnable(self,msg):
		enabled = Lepi.motor_get_enable(msg.port)
		return GetInt32Response(enabled)
		# print(msg)
	def srvMotorSetSpeed(self,msg):
		# print(msg)
		Lepi.motor_set_speed(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetSpeed(self,msg):
		# print(msg)
		speed = Lepi.motor_get_speed(msg.port)
		return GetInt32Response(speed)
	def srvMotorSetPosition(self,msg):
		print(msg)
		Lepi.motor_set_target_position(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetPosition(self,msg):
		print(msg)
		position = Lepi.motor_get_current_position(msg.port)
		return GetInt32Response(position)
	def pubButton(self,btn):
		if ButtonMap.has_key(btn):
			e = ButtonEvent()
			e.value = ButtonMap[btn]
			if 0x81 <= btn and btn <=0x89:
				e.type = 1
			elif 0x01 <= btn and btn <=0x09:
				e.type = 3
			self.pub_button_event.publish(e)
			return True
if __name__ == '__main__':
	rospy.init_node('pi_driver_node', anonymous=False)
	node = PiDriverNode()
	# rospy.on_shutdown(node.onShutdown)
	# thread.start_new_thread(camera_node.startCaptureRawCV, ())
	rospy.spin()
