#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import sys, select, termios, tty
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped
import rospy
import time
msg = """
Reading from the keyboard WSAD!
---------------------------
Moving around:
       i
   j    k    l

   m    ,    .

w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%


anything else : stop



CTRL-C to quit
"""
moveBindings = {
		'i':(1.0,0.0),  #axes[1]=v, axes[3]=steering
		'j':(0.0,1.0),
		'l':(0.0,-1.0),
		',':(-1.0,0.0),
		'm':(-1.0,1.0),
		'.':(-1.0,-1,0),


	       }

speedBindings={
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }
modeBindings={
		'0':('KB'),
		'1':('LANE')
	     }
class wasd_Drive:

    def __init__(self, verbose=False, debug=False ):
        self.mode = 'KB'
        # create a default object, no changes to I2C address or frequency
	# By default the address is 0x60 (see the stacking
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)

        self.verbose = verbose or debug
        self.debug = debug

        self.leftPWM = 0
        self.rightPWM = 0

        self.rightMotorMode = Adafruit_MotorHAT.FORWARD
        self.leftMotorMode = Adafruit_MotorHAT.FORWARD

        # Set local variable by reading parameters
        self.gain = rospy.get_param("~gain", 1.0)
        self.trim = rospy.get_param("~trim", -0.05)
        self.baseline = rospy.get_param("~baseline", 0.1)
        self.radius = rospy.get_param("~radius", 0.017)
        self.k = rospy.get_param("~k", 0.07)
        self.limit = rospy.get_param("~limit", 1.0)
        self.limit_max = 1.0
        self.limit_min = 0.0
        self.x = 0.0
        self.th = 0.0
        self.speed = rospy.get_param("~speed", 0.2)
        self.turn =rospy.get_param("~turn", 4.0)
        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmdCB)
        self.sub_stop_flag = rospy.Subscriber("~stop_flag", BoolStamped, self.stop_flagCB)
        #self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.stopFlag = False

    def getKey(self,settings):
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

    def setPWMvalue(self, key):
        if key == 'w':
             print 'w'
             self.rightMotorMode = Adafruit_MotorHAT.FORWARD
             self.leftMotorMode = Adafruit_MotorHAT.FORWARD
             self.leftPWM = 70
             self.rightPWM = 70
        elif key == 'a':
             print 'a'
	     self.rightMotorMode = Adafruit_MotorHAT.FORWARD
             self.leftMotorMode = Adafruit_MotorHAT.BACKWARD
             self.leftPWM = 70
             self.rightPWM = 70
        elif key == 's':
             print 's'
             self.rightMotorMode = Adafruit_MotorHAT.BACKWARD
             self.leftMotorMode = Adafruit_MotorHAT.BACKWARD
             self.leftPWM = 70
             self.rightPWM = 70
        elif key == 'd':
             print 'd'
             self.rightMotorMode = Adafruit_MotorHAT.BACKWARD
             self.leftMotorMode = Adafruit_MotorHAT.FORWARD
             self.leftPWM = 70
             self.rightPWM = 70

        elif key == 't':
             print 't'
             if self.leftPWM > 0:
	         self.leftPWM = self.leftPWM +5
             elif self.lefPWM < 0:
	         self.leftPWM = self.leftPWM -5
             if self.rightPWM > 0:
	         self.rightPWM = self.rightPWM +5
             elif self.rightPWM < 0:
	         self.rightPWM = self.rightPWM -5
        elif key == 'b':
             if self.leftPWM > 0:
	         self.leftPWM = self.leftPWM -5
             elif self.leftPWM < 0:
	         self.leftPWM = self.leftPWM +5
             if self.rightPWM > 0:
	         self.rightPWM = self.rightPWM -5
             elif self.rightPWM < 0:
	         self.rightPWM = self.rightPWM +5
             print 'b'
        else:
             self.leftPWM = 0
             self.rightPWM = 0
        self.updatePWM()

    def setkey(self, key):

        if key in modeBindings.keys():
           self.mode = modeBindings[key]
           print self.mode
        if self.mode != 'KB':
            return
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
        elif key in speedBindings.keys():
            self.speed= self.speed * speedBindings[key][0]
            if self.speed > self.limit_max:
                self.speed = self.limit_max
            self.turn=  self.turn * speedBindings[key][1]
            if self.turn > self.limit_max:
                self.turn = self.limit_max

            print self.speed, self.turn
        elif key == ' ' or key == 'k' :
            self.x = 0.0
            self.th = 0.0
        target_speed = self.x*self.speed
        target_angle = self.th*self.turn
	#print target_speed, target_angle
        [u_r_limited,u_l_limited] = self.inverse_kinematic([target_speed,target_angle])
	if u_l_limited > 0:
            self.leftPWM = int(255*u_l_limited)           
            self.leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif u_l_limited < 0:
            self.leftPWM = -1*int(255*u_l_limited)
            self.leftMotorMode = Adafruit_MotorHAT.BACKWARD
        elif u_l_limited == 0:
            self.leftPWM = 0
      
        if u_r_limited > 0:
	    self.rightPWM = int(255*u_r_limited)
            self.rightMotorMode = Adafruit_MotorHAT.FORWARD            
        elif u_r_limited < 0:
	    self.rightPWM = -1*int(255*u_r_limited)
            self.rightMotorMode = Adafruit_MotorHAT.BACKWARD
        elif u_r_limited == 0:
	    self.rightPWM = 0

   
        #print self.leftPWM,self.rightPWM
	self.updatePWM()


    def updatePWM(self):
        pwml = self.leftPWM
        pwmr = self.rightPWM
        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(self.leftMotorMode);
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(self.rightMotorMode);

    def inverse_kinematic(self,cmd):
        # assuming same motor constants k for both motors
	k_r = self.k
	k_l = self.k

	# adjusting k by gain and trim
	k_r_inv = (self.gain + self.trim) * k_r
	k_l_inv = (self.gain - self.trim) * k_l

	omega_r = (cmd[0] + 0.5 * cmd[1] * self.baseline) / self.radius
	omega_l = (cmd[0] - 0.5 * cmd[1] * self.baseline) / self.radius

	# conversion from motor rotation rate to duty cycle
	# u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
	u_r = omega_r * k_r_inv #0.029
	# u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
	u_l = omega_l * k_l_inv #0.031
        #print u_r,u_l
	# limiting output to limit, which is 1.0 for the duckiebot
	u_r_limited = max(min(u_r, self.limit), -self.limit)
	u_l_limited = max(min(u_l, self.limit), -self.limit)
        return [u_r_limited, u_l_limited]

    def car_cmdCB(self, msg_car_cmd):
        if self.mode == 'KB':
            return
        if self.stopFlag = True:
             #self.t_now = time.time()
             #if self.t_now - self.t_last < 1:
             #    return
             self.leftPWM = 0
             self.rightPWM = 0
             self.updatePWM()
             return
                  
        cmd = [msg_car_cmd.v, msg_car_cmd.omega]
        [u_r_limited,u_l_limited] = self.inverse_kinematic(cmd)
	if u_l_limited > 0:
            self.leftPWM = int(255*u_l_limited)           
            self.leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif u_l_limited < 0:
            self.leftPWM = -1*int(255*u_l_limited)
            self.leftMotorMode = Adafruit_MotorHAT.BACKWARD
        elif u_l_limited == 0:
            self.leftPWM = 0
      
        if u_r_limited > 0:
	    self.rightPWM = int(255*u_r_limited)
            self.rightMotorMode = Adafruit_MotorHAT.FORWARD            
        elif u_r_limited < 0:
	    self.rightPWM = -1*int(255*u_r_limited)
            self.rightMotorMode = Adafruit_MotorHAT.BACKWARD
        elif u_r_limited == 0:
	    self.rightPWM = 0
	self.updatePWM()

    def stop_flagCB(self, msg_flag):
        self.stopFlag = msg_flag.data

    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    do = True

    try:
         # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Constract BaseControl Obj
        rospy.loginfo("InoCar Base Control ...")
        dagu = wasd_Drive()
        
        print msg
	while do:
	    key = dagu.getKey(settings)
            if(key == '\x03'):
	        do = False
            
            dagu.setkey(key)
        rospy.spin()     
    finally:
        del dagu           

