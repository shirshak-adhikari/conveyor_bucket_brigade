#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
#pins for motor connection
in1 = 24
in2 = 23
en = 25
#switch to turn direction of motor
temp1 = 0

#pins for through beam sensor
BEAM_PIN_BACK = 4
BEAM_PIN_FRONT=26



GPIO.setmode(GPIO.BCM)
#setting up motor
GPIO.setwarnings(False)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
p = GPIO.PWM(en, 1000)
p.start(35)

GPIO.setup(BEAM_PIN_FRONT,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(BEAM_PIN_BACK,GPIO.IN,pull_up_down=GPIO.PUD_UP)

def main():

    def exit_gracefully(sig,frame):
        twist = Twist()
        pub.publish(twist)
        GPIO.cleanup()
        rospy.signal_shutdown("Stop")

    def break_beam_callback_FRONT(channel):
        if GPIO.input(BEAM_PIN_FRONT) :
    #        print("front beam unbroken")
    #        print("Stop Motor Turning")
    #        GPIO.output(in1, GPIO.LOW)
            pass

        else:
            print("front beam broken")
            print("Continue Motor Turning")
            GPIO.output(in1, GPIO.HIGH)


    def break_beam_callback_BACK(channel):
        if GPIO.input(BEAM_PIN_BACK):
            print("back beam unbroken")
            print("Continue Motor Turning")
            GPIO.output(in1, GPIO.HIGH)
        else:
            print("back beam broken")
            print("Stop Motor Turning")
            GPIO.output(in1, GPIO.LOW)


    #setting up sensor
    GPIO.setwarnings(False)
    GPIO.add_event_detect(BEAM_PIN_FRONT,GPIO.BOTH,callback=break_beam_callback_FRONT)
    GPIO.add_event_detect(BEAM_PIN_BACK,GPIO.BOTH,callback=break_beam_callback_BACK)



if __name__ == '__main__':
    rospy.init_node('Conveyor_control')
    signal.signal(signal.SIGINT,exit_gracefully)
    main()
    rospy.spin()
