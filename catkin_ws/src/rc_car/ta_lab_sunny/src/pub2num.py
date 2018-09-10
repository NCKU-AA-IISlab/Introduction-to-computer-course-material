#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int32
from std_msgs.msg import UInt16MultiArray

pub=rospy.Publisher('chatter', Int32, queue_size=10)
rospy.init_node('gen_int',anonymous=True)
rate= rospy.Rate(1)
while not rospy.is_shutdown():
	int_=[random.randint(0,101),random.randint(0,101)]
	#rospy.loginfo(int_)
	pub.publish(int_[0]+int_[1])
	rate.sleep()
