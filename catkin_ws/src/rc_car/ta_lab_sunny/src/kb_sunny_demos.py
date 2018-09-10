#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	
	
	#Initial Publisher
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	
	
	try:
		
		while(1):
			time_stamp = rospy.Time.now()
			#print time_stamp
			key = raw_input()
			if key == 'a':
				x=1
				th = 1
			elif key == 'w':
				x = 1
				th = 0
				
			elif key == 'd':				 
				
				x = 1
				th = -1
				
			elif key == 's':				 
				
				x = -1
				th = 0
			elif key == 'r':				 
				
				speed = speed * 1.1
				turn = turn * 1.1
				print vels(speed,turn)
			elif key == 'f':				 
				speed = speed * .9
				turn = turn * .9
				print vels(speed,turn)
			else:
				
				x = 0
				th = 0
				if (key == ' '):
					break

			
			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)
			
			
			
	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)		
		print 'no'
		
