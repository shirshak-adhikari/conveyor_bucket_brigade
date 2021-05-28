#!/usr/bin/env python
import rospy,signal
from shirshak_bucket_brigade.srv import bucket,bucketResponse
from shirshak_bucket_brigade.srv import align_robot,align_robotResponse #
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

roll = 0.0
pitch = 0.0
yaw =0.0
atfront =0.0
atback =5.0
x_position_2=0.0
x_position_1=0.0
x_position = 0.0
initial = 0





def exit_gracefully(sig,frame):
    twist = Twist()
    pub.publish(twist)
    rospy.signal_shutdown("Stop")

def yaw_generator(imu_data):
    global roll, pitch, yaw,x_position,y_position,initial_position_x,initial_position_y,initial
    orientation_q= imu_data.pose.pose.orientation
    position_v = imu_data.pose.pose.position
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    position_list = [position_v.x,position_v.y,position_v.z]
    (x_position,y_position,z_position) = position_list

    pub_position_x= rospy.Publisher("x_position",Float64,queue_size=10)
    pub_position_x.publish(x_position)


    if initial == True:
        reference_x_position = x_position
        initial = False


def range_filter(lidar_data):
    global atfront, atback
    atfront = lidar_data.ranges[0]
    atback = lidar_data.ranges[180]

def main():
    ## VARIABLES
    global twist,a,b,c,d,e,f,pub,initial
    state_a_b="a"
    state_c_d="c"
    state_f_e="e"
    twist = Twist()
    counter = 0


    retainer = None
    x_position_2=0.0
    x_position_1=0.0

    y_position_2=0.0
    y_position_1=0.0



    ##Publishers and Subscribers
    sub_orientation = rospy.Subscriber("odom",Odometry,yaw_generator)
    sub_range = rospy.Subscriber("scan",LaserScan,range_filter)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1,latch = False)
    reply = rospy.ServiceProxy('bucket_test',bucket)
    reply_control = rospy.ServiceProxy('control_robot',align_robot)


    pub.publish(twist)
    ##To shutdown bucket brigade
    signal.signal(signal.SIGINT,exit_gracefully)
    ##LOOP
    while not rospy.is_shutdown():
        if atback<0.5 and state_a_b =="a":
            srvc = reply(1)
            twist.linear.x= 0.1
            pub.publish(twist)
            print ("The robot moves now")
            state_a_b = "b"
            retainer = twist.linear.x
            initial = True # To measure initial reference postition of tbot

        elif atfront<0.5 and state_c_d =="c" and atfront!=0.0:
            print("The robot will go back now")
            srvc = reply(1)
            twist.linear.x = -0.1
            pub.publish(twist)
            state_c_d = "d"
            state_f_e = "f"
            counter =True
            x_position_2 = x_position
            # print("x_position_at_point2",x_position_2)
            retainer = twist.linear.x

        elif atback<0.5 and state_f_e == "f" and atback!=0.0:
            print("The robot will go front now")
            srvc = reply(1)
            twist.linear.x = 0.1
            pub.publish(twist)
            state_c_d = "c"
            state_f_e = "e"
            x_position_1 = x_position

            # print("x_position_at_point1",x_position_1)
            retainer = twist.linear.x


        difference_x = (x_position_2- x_position_1)*0.5
        pub_difference = rospy.Publisher("difference_x",Float64,queue_size=10)
        pub_difference.publish(difference_x)

        if round(difference_x,2) == round(x_position,2) and counter == True:
            pass
            # srvc = reply_control(0.0,0.0)
            # print("halfway")
            # twist.linear.x = retainer
            # print(twist.linear.x)
            # pub.publish(twist)
            # counter = False

        rospy.Rate(1).sleep()



if __name__=="__main__":
    rospy.init_node('BucketBrigade')
    main()
    rospy.wait_for_service('bucket_test')
    rospy.wait_for_service('control_robot')
    rospy.spin()
#try checking if y_value is different
