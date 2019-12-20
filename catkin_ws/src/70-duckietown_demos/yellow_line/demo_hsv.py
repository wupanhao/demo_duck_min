#!coding:utf-8
# 单线循迹，默认识别黄线，使用hsv颜色空间
import numpy as np
import argparse
import cv2
import time
import thread
# from image_rector import ImageRector

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
	if len(contours) == 1:
		return contours[0]
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
		# self.rector = ImageRector()
		self.car = CarDriver()
		self.captured = 0
		self.processed = 0
		self.speed = 0.8
		self.random_dir = -1
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	def start_capture(self,rate = 30):
		self.capture = cv2.VideoCapture(0)
		self.capture.set(cv2.CAP_PROP_FPS,60)
		self.last_update = time.time()
		print('capture start')
		self.capture_time = time.time()
		while self.active:
			ret,self.image = self.capture.read()
			end = time.time()
			self.captured = self.captured + 1
			self.last_update = end
			if self.captured % 150 == 0:
				print('captured %d frame in %.2f s' % (150,(time.time() - self.capture_time)))
				self.capture_time = time.time()
			# print('capture last update: %.2f ms' % ((end-self.last_update)*1000))	
			time.sleep(0.004)
		print('capture end')
		self.capture.release()
	def run_single_line(self):
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
			# image = self.rector.rect(image)
			image = cv2.resize(image,(320,240))
			image = cv2.flip(image,-1)
			image = image[120:140,:]
			# cnt_r = detect_cnt(image,[])
			cnt_y = detect_cnt(image,yellow_hsv)
			if cnt_y is not None:
				# print('yellow')
				# y_center,y_wh,y_angle = cv2.minAreaRect(cnt_y)
				# print(y_center,y_wh,y_angle)
				x,y,w,h = cv2.boundingRect(cnt_y)
				y_center = [x+w/2,y+h/2]
				cv2.drawContours(image, [cnt_y], -1, (0,255,255), thickness=2)
				offset = 160 - y_center[0]
				bias = offset/160.0
				self.speed = 0.9*(1-bias)
				if self.speed < 0.4:
					self.speed = 0.4
				left = self.speed*(1-bias)
				right = self.speed*(1+bias)
				self.car.setWheelsSpeed(left,right)
			else:
				bias = 0.05*self.random_dir
				left = self.speed*(1-bias)*0.5
				right = self.speed*(1+bias)*0.5			
				# self.car.setWheelsSpeed(left,right)
				self.car.setWheelsSpeed(0,0)
				self.random_dir = self.random_dir*-1
			cv2.imshow("images", image)
			# cv2.imshow("images", np.hstack([image, output]))
			c = cv2.waitKey(2)
			if c == 27:
				break

def test_cam():
	start = time.time()
	while True:
	# while(state < 2 or lost_count < 25):
		ret, image = capture.read()
		if ret == False:
			break;
		# 根据阈值找到对应颜色
		# image = rector.rect(image)
		# image = cv2.resize(image,(320,240))
		image = cv2.flip(image,-1)	
		end = time.time()
		print('last update: %.2f ms' % ((end-start)*1000))	
		start = end	
	# capture.release()	
		 
if __name__ == '__main__':
	demo_car = DemoCar()
	thread.start_new_thread(demo_car.start_capture, ())
	try:
		demo_car.run_single_line()
	except Exception as e:
		print(e)
		demo_car.active = False
		time.sleep(1)
		car.setWheelsSpeed(0,0)
	