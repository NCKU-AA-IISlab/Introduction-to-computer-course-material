#!/usr/bin/env python
import rospy
import numpy as np

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	
	
	#Initial Publisher  type:Twist
	#[BLANK]
	#Initial node 	
	#[BLANK]

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	th = 0
	
	
	try:
		
		while(1):
			#time_stamp = rospy.Time.now()
			#print time_stamp
			
			#Input 
			#[BLANK]

			
			#i: straight ;u: left turn; o:right ; k:back  ; r:speed up; f:speed down
			if 
				#[BLANK]
			elif 
				#[BLANK]
			else:
				
				x = 0
				th = 0
				if (key == ' '):
					break

			
			#Publish Topic			
			
			
			
			
	finally:
		#Publish Topic	0 command
				
		print 'shutdown'
		
