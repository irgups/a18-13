#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
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
			client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
			client.wait_for_server()

			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()

			goal.target_pose.pose.position.x = 4.9
			goal.target_pose.pose.position.y = 18.0
			goal.target_pose.pose.position.z = 0.0
			goal.target_pose.pose.orientation.z = 0.365
			goal.target_pose.pose.orientation.w = 0.931

			client.send_goal(goal)

			wait = client.wait_for_result()
			if wait:
				key = 2

if __name__ == '__main__': 
	try:	
		rospy.init_node("goals")
		rospy.Subscriber("cmd", String, callback)
		time_wait()
	except rospy.ROSInterruptException:
		pass
