# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image,CompressedImage
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import io
import numpy as np
import thread
from camera_utils import load_camera_info_2

class CameraNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.DIM = (640, 480)
		self.rate = 3
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		# initialize the camera and grab a reference to the raw camera capture
		self.camera = PiCamera()
		self.camera.resolution = self.DIM		
		self.camera.framerate = self.rate
		# allow the camera to warmup
		time.sleep(0.1)
		self.r = rospy.Rate(self.rate)

		self.rawCapture = PiRGBArray(self.camera, size=self.DIM)
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)

		self.stream = io.BytesIO()
		self.pub_compressed = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)

		self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)

		self.cali_file = '../camera_info/calibrations/default.yaml'
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		rospy.loginfo("[%s] CameraInfo: %s" % (self.node_name, self.camera_info_msg))
		K = np.array(self.camera_info_msg.K).reshape((3,3))
		# D = np.array(self.camera_info_msg.D[:4])
		D = np.array([0.,0.,0.,0.])
		# P = np.array(self.camera_info_msg.P).reshape((3,4))
		DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
		self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

		# frame = next( self.camera.capture_continuous(self.stream, format="jpeg", use_video_port=True) )
		# print(frame)

	def startCaptureRawCV(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			frame = next( self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True) )
			cv_image = frame.array

			image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
			image_msg.header.stamp = rospy.Time.now()
			image_msg.header.frame_id = self.frame_id
			self.pub_raw.publish(image_msg)			

			# '''
			rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR)  
			rect_image_msg = self.bridge.cv2_to_imgmsg(rect_image, "bgr8")
			rect_image_msg.header = image_msg.header
			self.pub_rect.publish(rect_image_msg)			
			# '''

			# self.r.sleep()
			self.rawCapture.truncate(0)
		self.camera.close()

	def startCaptureCompressed(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			gen = self.genCaptureCompressed()
			self.camera.capture_sequence(gen, format="jpeg", use_video_port=True, splitter_port=0)
		self.camera.close()

	def genCaptureCompressed(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			yield self.stream
			self.stream.seek(0)
			image_msg = CompressedImage()
			image_msg.format = "jpeg"
			image_msg.data = self.stream.getvalue()
			image_msg.header.stamp = rospy.Time.now()
			image_msg.header.frame_id = self.frame_id
			self.pub_compressed.publish(image_msg)			
			# self.decodeAndPublishRaw(image_msg)
			# self.r.sleep()
			self.stream.seek(0)
			self.stream.truncate()
	def decodeAndPublishRaw(self,image_msg):
		# Publish raw image
		np_arr = np.fromstring(image_msg.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		# time_1 = time.time()
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		# time_2 = time.time()
		img_msg.header.stamp = image_msg.header.stamp
		img_msg.header.frame_id = image_msg.header.frame_id
		self.pub_raw.publish(img_msg) 

	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." % (self.node_name))
		self.is_shutdown = True
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('camera_node', anonymous=False)
	camera_node = CameraNode()
	rospy.on_shutdown(camera_node.onShutdown)
	thread.start_new_thread(camera_node.startCaptureRawCV, ())
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	rospy.spin()