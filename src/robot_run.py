#!/usr/bin/env python
import roslaunch
import rospy
import time
from std_msgs.msg import String

key = 0

def callbackStart(msg):
	global key
	if msg.data == "START":
		key = 1
	if msg.data == "STOP":
		key = 2


def node_run():
	while not rospy.is_shutdown():
		global key
		if key == 1:
			key = 0
			launch.start()
			rospy.loginfo("started")
		elif key == 2:
			key = 0
			launch.shutdown()
			time.sleep(2)
			rospy.on_shutdown()

if __name__ == '__main__': 
	try:	
		rospy.init_node("node_run_launch_file_malina")
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nvidia/catkin_ws/src/autonet18p_Pi_sos/launch/start/start_fin.launch"])
		rospy.Subscriber("cmd", String, callbackStart)
		node_run()
	except rospy.ROSInterruptException:
		pass
