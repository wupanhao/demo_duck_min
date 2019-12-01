#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
import time
#!coding:utf-8
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from camera_utils import load_camera_info_2
from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections,GetApriltagDetectionsResponse
from apriltag_detector import ApriltagDetector
# from image_rector import ImageRector
from scipy.spatial.transform import Rotation as R

from usb_camera import UsbCamera
from pi_driver.srv import SetInt32,SetInt32Response,GetInt32,GetInt32Response,SetString,SetStringResponse

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.DIM = (640, 480)
		self.rate = 15
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(callback = self.PublishRaw)
		self.cv_image = None
		# allow the camera to warmup
		time.sleep(0.1)
		self.r = rospy.Rate(self.rate)
		# self.rector = ImageRector()
		self.detector = ApriltagDetector()
		self.visualization = True

		self.tag_detect_rate = 4
		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.camera_info_msg_rect = load_camera_info_2(self.cali_file)
		self.image_msg = Image()
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		self.pub_detections = rospy.Publisher("~image_detections", Image, queue_size=1)

		# self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)

		# self.pub_rect = rospy.Publisher("~rect/image", Image, queue_size=1)
		# self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)

		rospy.Service('~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)
		rospy.Service('~camera_set_enable', SetInt32, self.srvCameraSetEnable)
		rospy.Service('~camera_save_frame', SetInt32, self.srvCameraSetEnable)
		# self.timer_init = rospy.Timer(rospy.Duration.from_sec(1.0/self.tag_detect_rate), self.publish_rect)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def srvCameraSetEnable(self,params):
		if params.value == 1 and self.camera.active == False:
			ret = self.camera.open_camera()
			return SetInt32Response(0,ret)
		elif params.value == 0:
			self.camera.active = False
			return SetInt32Response(0,0)
		else:
			return SetInt32Response(1,1)
	def PublishRaw(self,cv_image):
		# Publish raw image
		self.cv_image = cv_image
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		# time_2 = time.time()
		img_msg.header.stamp = img_msg.header.stamp
		img_msg.header.frame_id = img_msg.header.frame_id
		self.pub_raw.publish(img_msg)
	def save_frame(self,params):
		try:
			directory = params.directory
			file_name = '%d.jpg' % (len(os.listdir(directory))+1)
			full_path = os.path.join(directory,file_name)
			cv2.imwrite(full_path,self.cv_image)
		except Exception as e:
			print(e)
		finally:
			pass

	def cbGetApriltagDetections(self,params):
		# print(params)
		if self.image_msg == None:
			return GetApriltagDetectionsResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('bgr_from_jpg: %s' % e)
				return
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
			# self.pub_raw.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			# rect_image = self.rector.rect(self.cv_image)
			rect_image = cv_image
			# self.pub_rect.publish(self.bridge.cv2_to_imgmsg(rect_image,"bgr8"))
			tags = self.detector.detect(rect_image)
			if self.visualization:
				for tag in tags:
					for idx in range(len(tag.corners)):
						cv2.line(rect_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
					# cv2.putText(rect_image, "中文测试", # not work
					cv2.putText(rect_image, str(tag.tag_id),
								org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
								fontFace=cv2.FONT_HERSHEY_SIMPLEX,
								fontScale=0.8,
								color=(0, 0, 255))
				imgmsg = self.bridge.cv2_to_imgmsg(rect_image,"bgr8")
				self.pub_detections.publish(imgmsg)			
			return self.toApriltagDetections(tags)
		return 
	def toApriltagDetections(self,tags):
		msg = GetApriltagDetectionsResponse()
		for tag in tags:
			r = R.from_dcm(np.array(tag.pose_R))
			offset = np.array(tag.pose_t)*100
			euler = r.as_euler('xyz', degrees=True)			
			detection = ApriltagPose(id=tag.tag_id,pose_r=euler,pose_t=offset)
			msg.detections.append(detection)
		return msg
	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." % (self.node_name))
		self.is_shutdown = True
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('camera_node', anonymous=False)
	camera_node = CameraNode()
	rospy.on_shutdown(camera_node.onShutdown)
	#thread.start_new_thread(camera_node.startCaptureRawCV, ())
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	rospy.spin()
