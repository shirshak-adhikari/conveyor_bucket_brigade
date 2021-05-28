#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from shirshak_bucket_brigade.srv import bucket,bucketResponse
from geometry_msgs.msg import Twist
from time import sleep

in1 = 24
in2 = 27
en = 23

BEAM_PIN_BACK = 22
BEAM_PIN_FRONT=17

GPIO.setmode(GPIO.BCM)
#setting up motor
GPIO.setwarnings(False)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
p = GPIO.PWM(en, 1000)
p.start(50)

GPIO.output(int1, GPIO.HIGH)


GPIO.setup(BEAM_PIN_BACK,GPIO.IN,pull_up_down=GPIO.PUD_UP)

def switch(value):
    global command
    pub.publish(command)
    if value.a==1:

     GPIO.output(in1,GPIO.HIGH)

    # rospy.loginfo(rospy.get_time())
    rospy.sleep(2)
    # rospy.loginfo(rospy.get_time())

    GPIO.output(in1,GPIO.LOW)
    GPIO.setwarnings(False)


    return bucketResponse(0)




rospy.init_node('bucket_server')
service = rospy.Service('bucket_test',bucket,switch)
pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
command = Twist()
rospy.spin()
