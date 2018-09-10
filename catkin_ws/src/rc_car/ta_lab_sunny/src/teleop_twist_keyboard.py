#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

0:A ,1:B, 2:X, 3:Y, 4:lb, 5:rb, 6:back, 7:start, 8:logitech, 9:left joy, 10:right joy

CTRL-C to quit
"""
msgButtons='''
Button
'''
moveBindings = {
		'w':(0,1,0,0,0,0,0,0),  #axes[1]=v, axes[3]=steering
		'd':(0,1,0,-1,0,0,0,0),
		'j':(0,0,0,1,0,0,0,0),
		'l':(0,0,0,-1,0,0,0,0),
		'a':(0,1,0,1,0,0,0,0),
		's':(0,-1,0,0,0,0,0,0),
		'.':(0,-1,0,0,1,0,0,0),
		'm':(0,-1,0,0,-1,0,0,0),
		
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }
modeBindings={
		'p':(1,0,0,0,0,0,0,0,0,0),	# manual mode	
		'o':(0,1,0,0,0,0,0,0,0,0),	# random mode
		'2':(0,0,1,0,0,0,0,0,0,0),
		'3':(0,0,0,1,0,0,0,0,0,0),
		'4':(0,0,0,0,1,0,0,0,0,0),
		'5':(0,0,0,0,0,1,0,0,0,0),
		'6':(0,0,0,0,0,0,1,0,0,0),
		'7':(0,0,0,0,0,0,0,1,0,0),
		'8':(0,0,0,0,0,0,0,0,1,0),
		'9':(0,0,0,0,0,0,0,0,0,1),
		
	     }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	#pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	pub_joy = rospy.Publisher('joy', Joy, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	flag=1
	button_active=0
	button_value=0
	axe= [0,0,0,0,0,0,0,0]
	button=[]
	
	try:
		#print msg
		#print vels(speed,turn)
		while(1):
			time_stamp = rospy.Time.now()
			#print time_stamp
			key = getKey()
			if key in moveBindings.keys():
				x=moveBindings[key][1]
				th=moveBindings[key][3]
				button_active = [0,0,0,0,0,0,0,0,0,0]
				
				
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
				
			elif key in modeBindings.keys():				 
				button_active = modeBindings[key]
				axe=[0,0,0,0,0,0,0,0]
				x = 0
				th = 0
				
				
			else:
				axe=[0,0,0,0,0,0,0,0]
				button_active = [0,0,0,0,0,0,0,0,0,0]
				x = 0
				th = 0
				if (key == '\x03'):
					break

			
			
			joy_msg =Joy()
			joy_msg.header.stamp=time_stamp
			joy_msg.axes=axe
			joy_msg.axes[1]=x*speed
			joy_msg.axes[3]=th*turn
			joy_msg.buttons=button_active
			pub_joy.publish(joy_msg)
			#print axe
			
	


	finally:
		
		joy_msg =Joy()
		joy_msg.header.stamp=time_stamp
		joy_msg.axes=[0,0,0,0,0,0,0,0]
		joy_msg.buttons=[0,0,0,0,0,0,0,0,0,0]
		pub_joy.publish(joy_msg)
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


