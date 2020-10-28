#!/usr/bin/env python
# -*- coding: utf-8 -*- позволяет писать русские комментарии 
import rospy
import roslib
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

global frame
global key_red

sample = "/home/ubuntu/catkin_ws/src/detector_signs/src/light.jpg"

sam = cv2.imread(sample)
sam = cv2.cvtColor(sam, cv2.COLOR_BGR2GRAY)
sam = cv2.resize(sam, (48,48))

#диапазон поиска цвета в HLS
red_color_low = (0,80,255)	
red_color_high = (255,255,255)
#диапазон поиска цвета в HSV
green_color_low = (50,100,200)	
green_color_high = (255,255,255)

red_flag = 0

key = 0
key_red = 0

rospy.init_node('light_node', anonymous=True)
pub = rospy.Publisher('light', Int16, queue_size=30)

def callback(data):
	global frame
	global key
	frame = CvBridge().imgmsg_to_cv2(data, "bgr8")	#захват кадра
	key = 1

def talker():
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		global frame
		global key
		global x
		global y
		global h
		global w

		if key == 1:	
			key = 0
			h, w, channels = frame.shape
			
			color_hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
			#преобразуем изображение в соответствуе с диапазоном
			only_red_hls = cv2.inRange(frame, red_color_low, red_color_high)
			#cv2.imshow('red', only_red_hls)
			'''
			kernel = np.ones((5,5),np.uint8)
			mask = cv2.erode(only_red_hls, None, iterations=2)
    			mask = cv2.dilate(mask, None, iterations=2) 
			#cv2.imshow('mask', mask)
			opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
			closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
			#cv2.imshow('closing', closing)
			'''
			im2, cnts, hierarchy = cv2.findContours(only_red_hls.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]

			#находим круг
			for c in cnts:
				# approximate the contour
				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.02 * peri, True)
				area = cv2.contourArea(c)
		
				#print area, len(approx)
			
				#ищем контур с площадью выше пороговой
				if area>1000 and area<2500 and len(approx) == 8:
				
					x,y,w,h = cv2.boundingRect(c)
					img=only_red_hls[y:y+h,x:x+w]
					img=cv2.resize(img, (48,48))
					#cv2.imshow('light', img)
					count = 0
					#сравниваем с шаблоном
					for i in range(48):
						for j in range(48):
							if sam[i][j]==img[i][j]:
								count+=1
					#print count
					if count > 1500:
						cv2.rectangle(frame, (x, y),(x + w, y + h), (0, 0, 255), 2)
						cv2.putText(frame, "RED", (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
						pub.publish(1)
						key_red = 1
						break
			if key_red == 1:
				global key_red
			
				img2=only_red_hls[y:y+h,x:x+w]
				img2=cv2.resize(img2, (48,48))
				#cv2.imshow('key', img2)
				count = 0
				for i in range(48):
					for j in range(48):
						if sam[i][j]==img2[i][j]:
							count+=1
				#print count
				if count < 500:
					pub.publish(0)
					key_red = 0
				
			'''
			color_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			#преобразуем изображение в соответствуе с диапазоном
			only_green_hls = cv2.inRange(color_hsv, green_color_low, green_color_high)
			#cv2.imshow('green', only_green_hls)
		
			im2, cnts, hierarchy = cv2.findContours(only_green_hls.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]

			#находим круг
			for c in cnts:
				#approximate the contour
				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.02 * peri, True)
				area = cv2.contourArea(c)
		
				#print area, len(approx)
		
				#ищем контур с площадью выше пороговой
				if area>1500 and area<2500 and len(approx) == 8:
					x,y,w,h = cv2.boundingRect(c)
					img=frame[y:y+h,x:x+w]
					#cv2.imshow('light', img)
					cv2.rectangle(frame, (x, y),(x + w, y + h), (0, 255, 0), 2)
					cv2.putText(frame, "GREEN", (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
					pub.publish(0)
					break
			'''
			#cv2.imshow('original', frame)
	
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		#rate.sleep()

if __name__ == '__main__':
	try:
		sub_video = rospy.Subscriber("video",Image, callback)
		talker()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()


