#!/usr/bin/env python

import rospy,signal
from shirshak_bucket_brigade.srv import bucket,bucketResponse
# from shirshak_bucket_brigade.srv import align_robot,align_robotResponse #
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

lasermsg = LaserScan()
atfront= tuple()
atback = tuple()
twist = 0


def callback_laser(value):
    global lasermsg,atfront, atback
    lasermsg = value
    atfront = lasermsg.ranges[0]
    atback = lasermsg.ranges[180]
    isinstance(atfront,tuple)
    isinstance(atback,tuple)


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
#################################################################################
#!/usr/bin/env python
import rospy,signal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from shirshak_bucket_brigade.srv import align_robot,align_robotResponse

roll = pitch = yaw = 0.0
kp=0.09
twist = 0

def exit_gracefully(sig,frame):
    twist = Twist()
    pub.publish(twist)
    rospy.signal_shutdown("Stop")

def get_rotation(msg):
    global roll, pitch, yaw
    a= True
    orientation_q= msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

def align_now(value):
    target_rad = value.a
    rate = rospy.Rate(10)
    while abs(target_rad -yaw)>0.05:
        # print abs(target-yaw)
        twist.angular.z= kp*(target_rad-yaw)
        pub.publish(twist)
        print("target={%f} current={%f} "%(round(target_rad,5),round(yaw,5)))
        rate.sleep()
    # while abs(target_rad -yaw)<0.15 and abs(target_rad -yaw)>0 :
    #     print abs(target_rad-yaw)
    #     twist.angular.z= 5*(target_rad-yaw)
    #     pub.publish(twist)
    #     print("target={} current={}",round(target_rad,5),round(yaw,5))
    #     rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)
    return align_robotResponse(0)


rospy.init_node('Controller')
sub = rospy.Subscriber("odom",Odometry,get_rotation)
pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
twist = Twist()
service =rospy.Service('control_robot',align_robot,align_now)
signal.signal(signal.SIGINT,exit_gracefully)
rospy.spin()
#need to call service in the mover code
