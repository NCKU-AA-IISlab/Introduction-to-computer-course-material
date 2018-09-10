#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped


class listener(object):
    def __init__(self):
    	
    	self.sub_chat = rospy.Subscriber('chatter', Int32, self.callback, queue_size=1)
    	self.sub_mode = rospy.Subscriber("mode", String, self.modecb, queue_size=1)
	self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist,self.cmdcb, queue_size=1)
	
	self.pub_car_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    	self.mode = "MANUAL"
	self.man_cmd = Twist()
	self.twist = Twist()
	self.speed = 0.5
	self.turn = 1.0
	#rospy.loginfo(" %s",self.mode)
	#self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.publishControl)
    def callback(self,msg):
        out = msg.data % 4
	#rospy.loginfo("I heard %s",out)
        if out == 2:
            x=1
            th = 1
        elif out == 0:
            x = 1
            th = 0
				
        elif out == 1:				 
            x = 1
            th = -1
				
        elif out == 3:				 
            x = -1
            th = 0
        else:
            x = 0
            th = 0
        self.twist.linear.x = x*self.speed; self.twist.linear.y =0; self.twist.linear.z = 0;
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = th*self.turn
        #rospy.loginfo( "%f", self.twist.linear.x)
        if self.mode == "RANDOM":
            car_cmd_msg = self.twist
            self.pub_car_cmd.publish(car_cmd_msg)
            print self.mode
            #rospy.loginfo( " %s",self.mode)
        else:
           return

    def modecb(self,data):
	self.mode = data.data
	#rospy.loginfo( " %s",self.mode)
	#rospy.loginfo(rospy.get_caller_id() + " %s",self.mode)
    	
    def cmdcb(self,data):
        self.man_cmd = data
        #rospy.loginfo( " %s",self.man_cmd)
        if self.mode == "MANUAL":
        	car_cmd_msg = self.man_cmd
            	self.pub_car_cmd.publish(car_cmd_msg)
            	print self.mode
            #rospy.loginfo( " %s",self.mode)
        else:
            return
    def publishControl(self,event):
        car_cmd_msg = Twist()
        car_cmd_msg.linear.x = 0; car_cmd_msg.linear.y = 0; car_cmd_msg.linear.z = 0;
        car_cmd_msg.angular.x = 0; car_cmd_msg.angular.y = 0; car_cmd_msg.angular.z = 0
        if self.mode == "MANUAL":
            car_cmd_msg = self.man_cmd
            self.pub_car_cmd.publish(car_cmd_msg)
            print self.mode
            #rospy.loginfo( " %s",self.mode)
        else:
            car_cmd_msg = self.twist
            self.pub_car_cmd.publish(car_cmd_msg)
            #rospy.loginfo( " %s",self.mode)
            #rospy.loginfo( "%f", car_cmd_msg.linear.x)
            print self.mode

if __name__ == '__main__':
        rospy.init_node('commander', anonymous=False)    
        cmder =listener()
        rospy.spin()
