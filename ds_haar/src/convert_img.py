#!/usr/bin/env python
# -*- coding: utf-8 -*- позволяет писать русские комментарии 
#можно уменьшить размер сжимания знака, например 64х64
#добавить выделение наибольшого контура
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

cap = cv2.VideoCapture(0)	#подключаем камеру

rospy.init_node('convert_node', anonymous=True)
rate = rospy.Rate(100)
pub_video = rospy.Publisher('/video', Image)

def talker():
	while not rospy.is_shutdown():
		ret, frame = cap.read()	#захват кадра
		
		#cv2.imshow('frame',frame) #показываем
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		
		_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
       		pub_video.publish(_frame)
		
		#rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		cap.release()
		pass

