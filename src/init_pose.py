#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

key = 0

def callback(msg):
	global key
	if msg.data == "START":
		key = 1

def time_wait():
	global key

	while not rospy.is_shutdown():
		if key == 1:
			pose = PoseWithCovarianceStamped()
			pose.header.frame_id = "map"
			pose.header.stamp = rospy.Time.now()

			pose.pose.pose.position.x = 19.5
			pose.pose.pose.position.y = 3.5
			pose.pose.pose.position.z = 0.0
			pose.pose.pose.orientation.z = -0.7
			pose.pose.pose.orientation.w = 0.7

			pub.publish(pose)

			key = 2
		
if __name__ == '__main__': 
	try:	
		rospy.init_node("init_pose_node")
		rospy.Subscriber("cmd", String, callback)
		pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=30)
		time_wait()
	except rospy.ROSInterruptException:
		pass
