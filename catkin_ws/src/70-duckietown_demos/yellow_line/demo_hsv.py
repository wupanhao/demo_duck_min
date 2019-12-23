#!coding:utf-8
# 单线循迹，默认识别黄线，使用hsv颜色空间
import numpy as np
import argparse
import cv2
import time
import thread
from image_rector import ImageRector

from Car import CarDriver
car = CarDriver()
# capture = cv2.VideoCapture(0)	
# fps = capture.get(cv2.CAP_PROP_FPS)
# print('frame rate : %d fps' % (fps) )
# capture.set(cv2.CAP_PROP_FPS,60)
# fps = capture.get(cv2.CAP_PROP_FPS)
# print('frame rate : %d fps' % (fps) )

red_hsv = [(np.array([0,160,50]),np.array([15,255,255])),(np.array([155,160,50]),np.array([180,255,255]))]
yellow_hsv = [(np.array([13,70,80]),np.array([55,255,255]))]
# green_hsv = [(np.array([46,160,50]),np.array([105,255,255]))]
green_hsv = [(np.array([56,160,46]),np.array([75,255,255]))]
white_hsv = [(np.array([0,0,221]),np.array([180,255,255])),(np.array([50,0,160]),np.array([95,255,255]))]

MIN_CNT_AREA = 200
MAX_CNT_AREA = 240*320/6

def detect_cnt(image,color_hsv):
	max_cnt = None	
	# ret,thresh1 = cv2.threshold(image,127,255,cv2.THRESH_BINARY)
	# ret,thresh2 = cv2.threshold(image,127,255,cv2.THRESH_BINARY_INV)
	# return findMaxCnt(ret)

	# change to hsv model
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# get mask
	# print(len(color_hsv))
	if len(color_hsv) == 1:
		color = color_hsv[0]
		mask = cv2.inRange(hsv, color[0], color[1])
	elif len(color_hsv) == 2:
		color1 = color_hsv[0]
		color2 = color_hsv[1]
		mask1 = cv2.inRange(hsv, color1[0], color1[1])
		mask2 = cv2.inRange(hsv, color2[0], color2[1])
		mask = cv2.bitwise_or(mask1, mask2) 
	else:
		return max_cnt
	return findMaxCnt(mask)

def findMaxCnt(mask):
	# 腐蚀操作
	# mask = cv2.erode(mask, None, iterations=2)
	# 膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
	# mask = cv2.dilate(mask, None, iterations=2)	
	_,contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# if len(contours) == 1:
	# 	return contours[0]
	max_cnt = None	
	max_area = 0
	cnts = []
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > MIN_CNT_AREA and area > max_area:
			max_cnt = cnt
	return max_cnt

class DemoCar(object):
	"""docstring for DemoCar"""
	def __init__(self):
		super(DemoCar, self).__init__()
		self.image = None
		self.active = True
		self.rector = ImageRector()
		self.car = CarDriver()
		self.captured = 0
		self.processed = 0
		self.speed = 0.8
		self.random_dir = -1
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	def start_capture(self,rate = 30):
		self.capture = cv2.VideoCapture(0)
		self.capture.set(cv2.CAP_PROP_FPS,30)
		self.last_update = time.time()
		print('capture start')
		self.capture_time = time.time()
		while self.active:
			ret,self.image = self.capture.read()
			end = time.time()
			self.captured = self.captured + 1
			self.last_update = end
			if self.captured % 150 == 0:
				# print('captured %d frame in %.2f s' % (150,(time.time() - self.capture_time)))
				self.capture_time = time.time()
			# print('capture last update: %.2f ms' % ((end-self.last_update)*1000))	
			time.sleep(0.004)
		print('capture end')
		self.capture.release()
	def follow_yellow(self):
		lost_count = 0
		start = time.time()
		while self.image is None:
			print('capture not start')
			time.sleep(1)
			# 根据阈值找到对应颜色
		last_update = self.last_update
		self.process_time = time.time()
		while True:
			if self.last_update == last_update:
				print('image not update , skip')
				time.sleep(0.005)
				continue
			# print('run last update %.2f ms' % (time.time() - self.last_update))
			self.processed = self.processed + 1
			if self.processed % 150 == 0:
				print('processed %d frame in %.2f s' % (150,(time.time() - self.process_time)))
				self.process_time = time.time()			
			image = self.image
			image = self.rector.rect(image)
			image = cv2.resize(image,(320,240))
			image_raw = cv2.flip(image,-1)
			image = image_raw[120:140,:]
			# cnt_r = detect_cnt(image,[])
			cnt_y = detect_cnt(image,yellow_hsv)
			if cnt_y is not None:
				# print('yellow')
				# y_center,y_wh,y_angle = cv2.minAreaRect(cnt_y)
				# print(y_center,y_wh,y_angle)
				x,y,w,h = cv2.boundingRect(cnt_y)
				y_center = [x+w/2,y+h/2]
				cv2.drawContours(image, [cnt_y], -1, (0,255,255), thickness=2)
				offset = 70 - y_center[0]
				bias = offset/160.0
				self.speed = 1.0*(1-bias)
				if self.speed < 0.8:
					self.speed = 0.8
				left = self.speed*(1-bias)
				right = self.speed*(1+bias)
				self.car.setWheelsSpeed(left,right)
				lost_count = 0
			else:
				bias = 0.05*self.random_dir
				left = self.speed*(1-bias)*0.5
				right = self.speed*(1+bias)*0.5			
				# self.car.setWheelsSpeed(left,right)
				self.car.setWheelsSpeed(0,0)
				self.random_dir = self.random_dir*-1
				# self.adjust_self()
				lost_count = lost_count + 1
				if lost_count > 15:
					print('yellow line interrupt! at red line')
					break
			image_raw[120:140,:] = image[:,:]
			cv2.imshow("images", image)
			# cv2.imshow("images", image_raw)
			# cv2.imshow("images", np.hstack([image, output]))
			c = cv2.waitKey(2)
			if c == 27:
				break
	def go_ahead_2_seconds(self):
		self.car.setWheelsSpeed(0.9,0.9)
		time.sleep(2)
		self.car.setWheelsSpeed(0,0)

	def go_ahead_and_stop(self):
		self.car.setWheelsSpeed(0.9,0.9)
		time.sleep(1.8)
		self.car.setWheelsSpeed(0,0)
	def go_ahead_and_detect_yellow(self):
		cnt_y = None
		self.car.setWheelsSpeed(0.6,0.6)
		while cnt_y is None:
			image = self.image
			image = self.rector.rect(image)
			image = cv2.resize(image,(320,240))
			image_raw = cv2.flip(image,-1)
			image = image_raw[120:140,:]
			# cnt_r = detect_cnt(image,[])
			cnt_y = detect_cnt(image,yellow_hsv)
			self.car.setWheelsSpeed(0.5,0.5)
		self.car.setWheelsSpeed(0.3,0.3)
	def turn_left(self):
		self.car.setWheelsSpeed(0.8,0.8)
		time.sleep(2)	
		self.car.setWheelsSpeed(0,0.8)
		time.sleep(1)
		self.car.setWheelsSpeed(0.8,0.8)
		time.sleep(1)
		car.setWheelsSpeed(0,0)	
	def turn_right(self):
		# self.car.setWheelsSpeed(0.5,0.5)
		# time.sleep(0.5)	
		self.car.setWheelsSpeed(0.8,0.3)
		time.sleep(1.5)
		car.setWheelsSpeed(0,0)	

	def adjust_self(self):
		image = self.image
		image = cv2.resize(image,(320,240))
		cnt_r = detect_cnt(image,yellow_hsv)
		if cnt_r is not None:
			print('red')
			r_center,r_wh,r_angle = cv2.minAreaRect(cnt_r)
			print(r_center,r_wh,r_angle)

def test_cam():
	demo_car = DemoCar()
	car.setWheelsSpeed(0,0)
	start = time.time()
	capture = cv2.VideoCapture(0)
	rector = ImageRector()
	while True:
		ret, image = capture.read()
		if ret == False:
			break;
		# 根据阈值找到对应颜色
		image = rector.rect(image)
		image = cv2.resize(image,(320,240))
		image = cv2.flip(image,-1)	
		end = time.time()
		print('last update: %.2f ms' % ((end-start)*1000))	
		start = end	
		cv2.imshow("images", image)
		# cv2.imshow("images", np.hstack([image, output]))
		c = cv2.waitKey(2)
		if c == 27:
			break
		 
def test_yellow():
	demo_car = DemoCar()
	thread.start_new_thread(demo_car.start_capture, ())
	try:
		demo_car.follow_yellow()
	except Exception as e:
		print(e)
		demo_car.active = False
		time.sleep(1)
		car.setWheelsSpeed(0,0)

def test_turn():
	demo_car = DemoCar()
	demo_car.car.setWheelsSpeed(0.5,0.5)
	time.sleep(2.5)	
	demo_car.car.setWheelsSpeed(0,0.4)
	time.sleep(2)
	demo_car.car.setWheelsSpeed(0.4,0.4)
	time.sleep(2)		
	car.setWheelsSpeed(0,0)


def airplane_to_train_station():
	demo_car = DemoCar()
	thread.start_new_thread(demo_car.start_capture, ())
	try:
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.turn_left()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.turn_right()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_and_stop()
	except Exception as e:
		print(e)
		demo_car.active = False
		time.sleep(1)
		car.setWheelsSpeed(0,0)	
	# demo_car.turn_right()

def train_station_to_airplane():
	demo_car = DemoCar()
	thread.start_new_thread(demo_car.start_capture, ())
	try:
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()		
		demo_car.turn_left()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.turn_right()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_2_seconds()
		demo_car.go_ahead_and_detect_yellow()
		demo_car.follow_yellow()
		demo_car.go_ahead_and_stop()
	except Exception as e:
		print(e)
		demo_car.active = False
		time.sleep(1)
		car.setWheelsSpeed(0,0)	
	# demo_car.turn_right()

if __name__ == '__main__':
	start = time.time()
	airplane_to_train_station()
	# train_station_to_airplane()
	end = time.time()
	print('time cost: %.2f s' % ((end-start)*1))	
	# test_cam()
