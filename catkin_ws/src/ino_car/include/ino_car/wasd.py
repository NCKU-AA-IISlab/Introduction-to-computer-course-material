#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import sys, select, termios, tty

msg = """
Reading from the keyboard WSAD!
---------------------------
Moving around:
       w    
   a    s    d



t : up (+z)
b : down (-z)

anything else : stop



CTRL-C to quit
"""


class wasd_Drive:

    def __init__(self, verbose=False, debug=False, ):
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
           
    def updatePWM(self):
        pwml = self.leftPWM
        pwmr = self.rightPWM
        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(self.leftMotorMode);
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(self.rightMotorMode);

    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    dagu = wasd_Drive()
    do = True
    try:
        print msg
	while do:
	    key = dagu.getKey(settings)
            if(key == '\x03'):
	        do = False
            dagu.setPWMvalue(key)

    finally:
        del dagu           

