#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Sunny Tseng<sunny567886@gmail.com>
#          
#          

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DAGU_Differential_Drive:

    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  

    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  

    AXEL_TO_RADIUS_RATIO = 1.0     # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2;       # speed tolerance level
    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False, ):

        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)


        self.verbose = verbose or debug
        self.debug = debug
        
        self.left_sgn = 1.0;
        if left_flip:
            self.left_sgn = -1.0;

        self.right_sgn = 1.0;
        if right_flip:
            self.right_sgn = -1.0;

        self.speed = 0.0
        self.angle = 0.0

        self.leftSpeed = 0.0
        self.rightSpeed = 0.0

    def TestForward(self,value):
        pwml = value
        pwmr = value
	self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(Adafruit_MotorHAT.FORWARD);
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(Adafruit_MotorHAT.FORWARD);


    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat



if __name__ == '__main__':

    from time import sleep
    dagu = DAGU_Differential_Drive()
    try:
        while True:
           duty_s = raw_input("PWM value(0~255):") 
           if duty_s == 'p':
                do =False
                break

           duty = int(duty_s)
           dagu.TestForward(duty)
    except KeyboardInterrupt:
        pass

    del dagu
