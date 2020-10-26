#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped 

def time_wait():
	key = 0
	while not rospy.is_shutdown():
		if key == 0:
			pose = PoseWithCovarianceStamped()
			pose.header.frame_id = "map"
	  		pose.header.stamp = rospy.Time.now()

	  		pose.pose.pose.position.x = 17.5
	  		pose.pose.pose.position.y = 19.5
	 		pose.pose.pose.position.z = 0.0
	  		pose.pose.pose.orientation.z = 0.0
	  		pose.pose.pose.orientation.w = 1.0

			pub.publish(pose)
			
			key = 1
		
if __name__ == '__main__': 
	try:	
		rospy.init_node("init_pose_node")
		pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=30)
		time_wait()
	except rospy.ROSInterruptException:
		pass
