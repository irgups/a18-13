#!/usr/bin/env python
# -*- coding: utf-8 -*- позволяет писать русские комментарии 
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/robofest2020_ds.avi',fourcc, 25.0, (640,480))

global key
key = 0

detect = False  # переменная для выделения знаков
area = 1500  # пороговая площадь для обнаружения знаков

# пороги маски цвета
blue_color_low = (20, 80, 50)  # blue_color_low = (74,1,123)
blue_color_high = (255, 255, 255)  # blue_color_high = (255,255,255)
# пороги маски красного цвета
red_color_low = (0, 100, 40)
red_color_high = (230, 255, 255)

# путь к каскаду
# путь к каскадам
cas_f = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_f_20x20.xml'
cas_l = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_left_20x20.xml'
cas_r = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade//cascade_right_20x20.xml'
cas_f_or_l = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_f_or_l_20x20.xml'
cas_f_or_r = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_f_or_r_20x20.xml'
cas_stop = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_stop_20x20.xml'
cas = '/home/one/catkin_ws/src/autonet18p_Pi_sos/ds_haar/src/cascade/cascade_20x20.xml'

cascade = cv2.CascadeClassifier(cas)
cascade_f = cv2.CascadeClassifier(cas_f)
cascade_l = cv2.CascadeClassifier(cas_l)
cascade_r = cv2.CascadeClassifier(cas_r)
cascade_f_or_l = cv2.CascadeClassifier(cas_f_or_l)
cascade_f_or_r = cv2.CascadeClassifier(cas_f_or_r)
cascade_stop = cv2.CascadeClassifier(cas_stop)

rospy.init_node('detector_node', anonymous=True)
# pub_traffic_signs = rospy.Publisher('traffic_signs', String)

# порог обнаружения знака
def th(x):
    return {
        'forward': 1300,
        'left': 1100,
        'right': 1100,
        'f_or_l': 1100,
        'f_or_r': 1100,
        'stop': 1300,
    }[x]


def putDet(t):
    global x
    global y
    global w
    global h

    print(t)
    cv2.putText(frame, t, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (125, 0, 255),
                  2)  # выделяем обнаруженый знак
    # pub_traffic_signs.publish(t)


def callback(data):
    global key
    global frame
    frame = CvBridge().imgmsg_to_cv2(data, "bgr8")  # захват кадра
    frame = cv2.flip(frame, 0)
    key = 1
    print("yesyesyes")

def detector():
    global frame
    global key
    global x
    global y
    global w
    global h

    while not rospy.is_shutdown():
        if key == 1:

            key = 0
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # делаем его ч/б
            sign_cas = cascade.detectMultiScale(gray, 1.3, 1)  # применяем каскад

            for (x, y, w, h) in sign_cas:
                roi = gray[y:y + h, x:x + w]  # вырезаем детектированный объект
                height, width = roi.shape  # получаем размеры детектированного объекта
                ar = height * width
                # print ar
                # если площадь детектированного объекта больше порогового, то рассматриваем дальше что это
                if ar >= area and ar <= area * 4:
                    # cv2.imshow('roi', roi)	#показываем вырезанный знак

                    sign_cas_f = cascade_f.detectMultiScale(roi, 1.3, 1)

                    if sign_cas_f != ():
                        detect = True
                        putDet("forward")
                        break
                    else:
                        sign_cas_l = cascade_l.detectMultiScale(roi, 1.3, 1)
                        if sign_cas_l != ():
                            detect = True
                            putDet("left")
                            break
                        else:
                            sign_cas_r = cascade_r.detectMultiScale(roi, 1.3, 1)
                            if sign_cas_r != ():
                                detect = True
                                putDet("right")
                                break
                            else:
                                sign_cas_f_or_l = cascade_f_or_l.detectMultiScale(roi, 1.3, 1)
                                if sign_cas_f_or_l != ():
                                    detect = True
                                    putDet("f_or_l")
                                    break
                                else:
                                    sign_cas_f_or_r = cascade_f_or_r.detectMultiScale(roi, 1.3, 1)
                                    if sign_cas_f_or_r != ():
                                        detect = True
                                        putDet("f_or_r")
                                        break
                                    else:
                                        sign_cas_stop = cascade_stop.detectMultiScale(roi, 1.3, 1)
                                        if sign_cas_stop != ():
                                            detect = True
                                            putDet("stop")
                                            break
                                        else:
                                            detect = False
                                            break

            #cv2.imshow('original', frame)
            out.write(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # _frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
        # pub_video.publish(_frame)

        # rate.sleep()


if __name__ == '__main__':
	try:
		sub_video = rospy.Subscriber("/cam_top", Image, callback)
		detector()
	except rospy.ROSInterruptException:
		out.release()
	pass
