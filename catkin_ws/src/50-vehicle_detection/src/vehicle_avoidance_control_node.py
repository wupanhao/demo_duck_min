#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose, Pose2DStamped

import os
import rospkg
import rospy
import yaml
import time
from twisted.words.protocols.oscar import CAP_SERV_REL
from math import sqrt, sin, cos

from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image,CameraInfo

from camera_utils import load_camera_info_2

from scipy.spatial.transform import Rotation as R

from apriltags3 import Detector


class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospack = rospkg.RosPack()
		self.car_cmd_pub = rospy.Publisher("~car_cmd",
				Twist2DStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size=1)
		self.subscriber = rospy.Subscriber("~detection",
				BoolStamped, self.callback,  queue_size=1)
		self.sub_vehicle_pose = rospy.Subscriber("~vehicle_pose", VehiclePose, self.cbPose, queue_size=1)
		self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)

		self.config	= self.setupParam("~config", "baseline")
		self.cali_file_name = self.setupParam("~cali_file_name", "default")

		self.cali_file = rospack.get_path('duckietown') + \
				"/config/" + self.config + \
				"/vehicle_detection/vehicle_avoidance_control_node/" +  \
				self.cali_file_name + ".yaml"
		if not os.path.isfile(self.cali_file):
			rospy.logwarn("[%s] Can't find calibration file: %s.\n"
					% (self.node_name, self.cali_file))
		self.loadConfig(self.cali_file)
		self.controllerInitialization()
		self.detection_prev=None

		self.active = True
		self.waited = False
		self.bridge = CvBridge()
		self.last_stamp = rospy.Time.now()
		self.publish_freq = 2
		self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
		# self.sub_compressed_img = rospy.Subscriber("/ubiquityrobot/camera_node/image/compressed",CompressedImage,self.cbImg,queue_size=1)
		self.sub_raw_img = rospy.Subscriber("/ubiquityrobot/camera_node/image_raw",Image,self.cbRawImg,queue_size=1)

		self.camera_info_msg = load_camera_info_2(rospkg.RosPack().get_path('duckietown') +  '/duckiefleet/calibrations/camera_intrinsic/ubiquityrobot.yaml')
		K = np.array(self.camera_info_msg.K).reshape((3,3))
		# D = np.array(self.camera_info_msg.D[:4])
		D = np.array([0.,0.,0.,0.])
		# P = np.array(self.camera_info_msg.P).reshape((3,4))
		DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
		self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
		self.detector = Detector(searchpath=['/home/ubuntu/workspace/apriltags3-py/apriltags/lib'],
			families='tag36h11',
			nthreads=1,
			quad_decimate=3.0,
			quad_sigma=0.0,
			refine_edges=1,
			decode_sharpening=0.25,
			debug=0)
		self.pub_tag = rospy.Publisher("~tag_detections",Image,queue_size=1)
		self.pub_rect = rospy.Publisher("/ubiquityrobot/camera_node/image/rect",Image,queue_size=1)
		self.pub_camera_info = rospy.Publisher("/ubiquityrobot/camera_node/image/camera_info", CameraInfo, queue_size=1)
		self.cameraMatrix = np.array(self.camera_info_msg.K).reshape((3,3))
		self.camera_params = ( self.cameraMatrix[0,0], self.cameraMatrix[1,1], self.cameraMatrix[0,2], self.cameraMatrix[1,2] )		
		self.visualization = True
		self.speed_up_factor = 1.2
		self.sleep_time = rospy.Time.now()
# 		self.v_gain = 1
# 		self.vehicle_pose_msg_temp = VehiclePose()
# 		#self.vehicle_pose_msg_temp = Pose2DStamped()
# 		self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
# 		#self.time_temp = rospy.Time.now()
# 		self.v_rel = 0
# 		self.v = 0
# 		self.detection = False
# 		self.v_error_temp = 0
# 		self.I = 0
# 		self.v_follower = 0
# 		self.rho_temp = 0
# 		self.omega = 0

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def loadConfig(self, filename):
		stream = file(filename, 'r')
		data = yaml.load(stream)
		stream.close()
		self.desired_distance = data['desired_distance']
		rospy.loginfo('[%s] desired_distance : %s' % (self.node_name,
				self.desired_distance))
		self.minimal_distance = data['minimal_distance']
		rospy.loginfo('[%s] minimal_distance : %s' % (self.node_name,
				self.minimal_distance))
		self.Kp = data['Kp']
		rospy.loginfo('[%s] Kp : %s' % (self.node_name,
				self.Kp))
		self.Ki = data['Ki']
		rospy.loginfo('[%s] Ki : %s' % (self.node_name,
				self.Ki))
		self.Kd = data['Kd']
		rospy.loginfo('[%s] Kd : %s' % (self.node_name,
				self.Kd))
		self.Kp_delta_v = data['Kp_delta_v']
		rospy.loginfo('[%s] Kp_delta_v : %s' % (self.node_name,
				self.Kp_delta_v))

	def controllerInitialization(self):
		self.vehicle_pose_msg_temp = VehiclePose()
		self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
		self.time_temp = rospy.Time.now()
		self.v_rel = 0
		self.v = 0
		self.detection = False
		self.v_error_temp = 0
		self.I = 0
		self.v_follower = 0
		#self.rho_temp = 0
		self.omega = 0

	def callback(self, data):

		vehicle_detected_msg_out = BoolStamped()
		vehicle_detected_msg_out.header.stamp = data.header.stamp
		vehicle_detected_msg_out.data = data.data
		self.vehicle_detected_pub.publish(vehicle_detected_msg_out)
		self.detection_prev=self.detection
		self.detection = data.data

		if  not data.data:
			#self.v_gain = 1
			#self.P = 0
			self.I = 0


	def cbPose(self, vehicle_pose_msg):
# 		desired_distance = 0.3
		#distance_error_tolerance = 0.04
# 		d_min = 0.2
# 		Kp = 0.7
# 		Kp_delta_v = 0.8
# 		Ki = 0.0
# 		Kd = 0.00

		time = rospy.Time.now()

		#Ts = (vehicle_pose_msg.header.stamp - self.vehicle_pose_msg_temp.header.stamp).to_sec()
		Ts = (time - self.time_temp).to_sec()
# 		print("-----")
#  		print(Ts)
# 		print(Ts2)
		self.vehicle_pose_msg_temp.header.stamp = vehicle_pose_msg.header.stamp
		#print(Ts)
		if Ts > 4:
			self.v_rel = 0
			if vehicle_pose_msg.rho.data < self.minimal_distance:
				self.v = 0
			else:
				self.v = self.v_follower
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			self.v_error_temp = 0
			self.I = 0
		else:
			self.v_rel = (vehicle_pose_msg.rho.data - self.vehicle_pose_msg_temp.rho.data)/Ts
			v_leader = self.v_follower + self.v_rel
			delta_v = (vehicle_pose_msg.rho.data - self.desired_distance)/Ts * self.Kp_delta_v
			v_des = v_leader + delta_v

# 			print("v leader")
# 			print(v_leader)
# 			print("v follower")
# 			print(self.v_follower)
# 			print("delta v")
# 			print(delta_v)
# 			print("v_rel")
# 			print(self.v_rel)
# 			print("rho")
# 			print(vehicle_pose_msg.rho)
# 			print("psi")
# 			print(vehicle_pose_msg.psi)

			v_error = v_des - self.v_follower

			self.P = self.Kp*v_error
			self.I = self.I + self.Ki * (v_error + self.v_error_temp)/2.0*Ts
			self.D = self.Kd * (v_error + self.v_error_temp)/Ts
			self.v = self.P + self.I + self.D

			if self.v < 0 or vehicle_pose_msg.rho.data < self.minimal_distance:
				self.v = 0

			#self.rho_temp = rho
			self.v_error_temp = v_error
			self.v_temp = self.v
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			#print(self.v)

		self.time_temp = time

# 		v_gain_max = 1.5
# 		if d_min > vehicle_pose_msg.rho.data:
# 			self.v_gain = 0
# 		else:
# 			self.v_gain = (vehicle_pose_msg.rho.data - d_min)/(d_desired - d_min)
# 			if self.v_gain > v_gain_max:
# 				self.v_gain = v_gain_max

	def cbCarCmd(self, car_cmd_msg):
		if not self.active or (self.sleep_time > rospy.Time.now() ):
			#self.pubStop()
			return
		car_cmd_msg_current = Twist2DStamped()
		car_cmd_msg_current = car_cmd_msg
		car_cmd_msg_current.header.stamp = rospy.Time.now()
		if self.detection:
			car_cmd_msg_current.v = self.v
			if self.v == 0:
				car_cmd_msg_current.omega = 0
			#print(self.v)

		if self.detection_prev and not self.detection:
			car_cmd_msg_current.v=0

		if car_cmd_msg_current.v>=0.25:
			car_cmd_msg_current.v=0.25
		car_cmd_msg_current.v = car_cmd_msg_current.v * self.speed_up_factor
		self.car_cmd_pub.publish(car_cmd_msg_current)
		#print(self.v_gain)

# v : speed
# omega : steer , left:-8.3,right:8.3
	def publishCmd(self,v=0.0,omega=0.0):
		cmd_msg = Twist2DStamped()
		cmd_msg.header.stamp = rospy.Time.now()
		cmd_msg.v = v
		cmd_msg.omega = omega
		print(v,omega)
		self.car_cmd_pub.publish(cmd_msg)

	def turn_for_100ms(self,v=0.0,omega=0.0,count = 10 ):
		print('turn_for_100ms')
		for i in range(count):
			self.publishCmd(v,omega)
			time.sleep(0.1)

	def cbRawImg(self,msg):
		now = rospy.Time.now()
		time_pase = now - self.last_stamp
		if time_pase < self.publish_duration:
			return
		else:
			self.last_stamp = now
			if self.sleep_time > now:
				return
		time_0 = time.time()
		cv_image = self.bridge.imgmsg_to_cv2(msg)
		time_1 = time.time()
		rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR) 
		time_2 = time.time()
		rect_imgmsg = self.bridge.cv2_to_imgmsg(rect_image,"bgr8")
		self.pub_rect.publish(rect_imgmsg)
		self.camera_info_msg.header = rect_imgmsg.header
		self.pub_camera_info.publish(self.camera_info_msg)
		# print('[image rect] decode:%f rect:%f ' % ( time_1- time_0, time_2 - time_1 ))
		# return
		image_gray = cv2.cvtColor(rect_image,cv2.COLOR_BGR2GRAY)
		tags = self.detector.detect(image_gray, True, self.camera_params, 0.041) # tag size in meter
		time_3 = time.time()
		if self.visualization:
			for tag in tags:
				for idx in range(len(tag.corners)):
					cv2.line(cv_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
				cv2.putText(cv_image, str(tag.tag_id),
							org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
							fontFace=cv2.FONT_HERSHEY_SIMPLEX,
							fontScale=0.8,
							color=(0, 0, 255))
			imgmsg = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
			self.pub_tag.publish(imgmsg)
			self.camera_info_msg.header.stamp = msg.header.stamp
		time_4 = time.time()
		print('[apriltag detect] convert:%f rect:%f detect:%f publish:%f' % ( time_1- time_0, time_2 - time_1 , time_3- time_2, time_4 - time_3 ))
		if len(tags) > 0:
			tag = tags[0]
			base = np.eye(3)
			r = R.from_dcm(np.array(tag.pose_R))
			offset = np.array(tag.pose_t)*100
			euler = r.as_euler('xyz', degrees=True)
			print('tag id %d wx:%f wy:%f wz:%f x:%f y:%f z:%f'% ( tag.tag_id , euler[0], euler[1],euler[2] ,offset[0],offset[1],offset[2] ) )
			if offset[2] > 15:
				return
			if tag.tag_id == 5 and not self.waited: # Wait 5 seconds
				self.speed_up_factor = 2
				self.sleep_time = rospy.Time.now()+rospy.Duration.from_sec(1.0*5)
				self.active = True
				self.waited = True
			elif tag.tag_id == 4: # Faster speed
				#self.speed_up_factor = 3.0
				self.active = True
                        elif tag.tag_id == 3: # turn left
                                #self.speed_up_factor = 1.2
				if self.waited:
					return
                                self.active = False
				self.waited = True
                                self.turn_for_100ms(0.3,0.0,25)
                                self.turn_for_100ms(0.3,2,25)
                                self.turn_for_100ms(0.3,0.0,15)
                                #self.turn_for_100ms(0.3,0.0,35)
				self.active = True
				time.sleep(3)
				self.active = False
                                self.turn_for_100ms(0.3,-2.0,25)
                                self.active = True
                        elif tag.tag_id == 2: # stop
                                #self.speed_up_factor = 1.7
				if not self.waited:
					return
                                self.active = False
                                self.waited = False 
                                self.turn_for_100ms(0.4,0,20)
                                self.turn_for_100ms(0,0,5)
			elif tag.tag_id == 1: 
				self.active = False
				# self.speed_up_factor = 0
		else:
			print('no tag detected')

	def pubStop(self):
		msg = Twist2DStamped()
		msg.v = 0
		msg.omega = 0
		self.car_cmd_pub.publish(msg)	
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()
