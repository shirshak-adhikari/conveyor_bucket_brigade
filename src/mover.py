#!/usr/bin/env python

import rospy,signal
from shirshak_bucket_brigade.srv import bucket,bucketResponse
# from shirshak_bucket_brigade.srv import align_robot,align_robotResponse #
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

lasermsg = LaserScan()
atfront= 0
atback = 0
twist = 0


def callback_laser(value):
    global lasermsg,atfront, atback
    lasermsg = value
    atfront = lasermsg.ranges[0]
    atback = lasermsg.ranges[180]


def exit_gracefully(sig,frame):
    twist = Twist()
    pub.publish(twist)
    rospy.signal_shutdown("Stop")

# def initial_yaw_reference():
#     global initial_roll, initial_pitch, initial_yaw
#     a= True
#     orientation_q= msg.pose.pose.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
#     print initial_yaw



def main():

    global lasermsg, twist,pub,a,b,c,d,e,f
    state_a_b="a"
    state_c_d="c"
    state_f_e="e"
    run = False
    sub = rospy.Subscriber('scan',LaserScan,callback_laser)
    # sub = rospy.Subscriber("odom",Odometry,initial_yaw_reference)

    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    twist = Twist()
    reply = rospy.ServiceProxy('bucket_test',bucket)
    # reply_align = rospy.ServiceProxy('control_robot',align_robot)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if atback<0.2 and state_a_b == "a":
            srvc = reply(1)
            twist.linear.x= 0.1
            pub.publish(twist)
            print ("The robot moves now")
            state_a_b = "b"


        elif atfront<0.2 and state_c_d =="c" and atfront!=0.0:
            srvc = reply(1)
            twist.linear.x = -0.1
            pub.publish(twist)
            print ("The robot will go back")
            state_c_d = "d"
            state_f_e = "e"

        elif atback<0.2 and state_f_e == "e" and atback!=0.0:
            srvc = reply(1)
            twist.linear.x = 0.1
            pub.publish(twist)
            print("The robot will go back now")
            state_c_d = "c"
            state_f_e = "f"


            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Mover')
    rospy.wait_for_service('bucket_test')
    # rospy.wait_for_service('control_robot')
    signal.signal(signal.SIGINT,exit_gracefully)
    main()
    rospy.spin()
