#!/usr/bin/env python
import rospy,signal
from shirshak_bucket_brigade.srv import bucket,bucketResponse
from shirshak_bucket_brigade.srv import align_robot,align_robotResponse #
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

roll = 0.0
pitch = 0.0
yaw =0.0
atfront =0.0
atback =0.0


def exit_gracefully(sig,frame):
    twist = Twist()
    pub.publish(twist)
    rospy.signal_shutdown("Stop")

def yaw_generator(imu_data):
    global roll, pitch, yaw
    orientation_q= imu_data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

def range_filter(lidar_data):
    global atfront, atback
    atfront = lidar_data.ranges[0]
    atback = lidar_data.ranges[180]
    print (atback)
def main():
    ## VARIABLES
    global twist,a,b,c,d,e,f,pub
    state_a_b="a"
    state_c_d="c"
    state_f_e="e"
    twist = Twist()

    ##Publishers and Subscribers
    sub_orientation = rospy.Subscriber("odom",Odometry,yaw_generator)
    sub_range = rospy.Subscriber("scan",LaserScan,range_filter)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1,latch = False)
    reply = rospy.ServiceProxy('bucket_test',bucket)

    ##To shutdown bucket brigade
    signal.signal(signal.SIGINT,exit_gracefully)
    ##LOOP
    twist.linear.x = 0.0
    pub.publish(twist)

    while not rospy.is_shutdown():
        if atback<0.2 and state_a_b =="a":
            srvc = reply(1)
            twist.linear.x= 0.1
            pub.publish(twist)
            print ("The robot moves now")
            state_a_b = "b"
        elif atfront<0.2 and state_c_d =="c" and atfront!=0.0:
            print("The robot will go back now")
            srvc = reply(1)
            twist.linear.x = -0.1
            pub.publish(twist)
            state_c_d = "d"
            state_f_e = "f"
        elif atback<0.2 and state_f_e == "f" and atback!=0.0:
            print("The robot will go front now")
            srvc = reply(1)
            twist.linear.x = 0.1
            pub.publish(twist)
            state_c_d = "c"
            state_f_e = "e"

        rospy.Rate(10).sleep()



if __name__=="__main__":
    rospy.init_node('BucketBrigade')
    main()
    rospy.wait_for_service('bucket_test')

    rospy.spin()
