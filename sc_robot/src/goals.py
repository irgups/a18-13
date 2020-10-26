#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def time_wait():
	key = 0
	while not rospy.is_shutdown():
		if key == 0:
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
				key = 1

if __name__ == '__main__': 
	try:	
		rospy.init_node("goals")
		time_wait()
	except rospy.ROSInterruptException:
		pass
