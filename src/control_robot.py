#!/usr/bin/env python
import rospy,signal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from shirshak_bucket_brigade.srv import align_robot,align_robotResponse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from std_msgs.msg import Float64


roll = pitch = yaw = 0.0
###Control Parameters
kp=0.9
ki = 0
kd = 0.0
bias = 0
error_for_controller =0
integral = 0

iteration_time = 0
derivative = 0
seconds = 0


# Controls parameters end
twist = 0
integral

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

    target_rad = round(value.a,2)
    rate = rospy.Rate(1)
    seconds_prior = 0
    integral_prior = 0
    error_prior = 0
    error_for_controller = target_rad-yaw
    # while abs(target_rad-round(yaw,3))>0.05:
    while abs(error_for_controller)>0.01:
        print("s")
        seconds = rospy.get_time()
        iteration_time = seconds-seconds_prior
        # print(iteration_time)
        error_for_controller = target_rad-yaw
        integral = integral_prior + error_for_controller*iteration_time
        derivative = (error_for_controller-error_prior)/iteration_time
        twist.angular.z= kp*error_for_controller + ki*integral + kd*derivative + bias
        pub.publish(twist)
        pub_kp.publish(kp*error_for_controller)
        # pub_ki.publish(ki*integral)
        # pub_kd.publish(kd*derivative)
        print("target={%f} current={%f} "%(round(target_rad,5),round(yaw,5)))
        error_prior = error_for_controller
        integral_prior = integral
        seconds_prior = seconds
        rate.sleep()
    print("Reoreintation done")
    integral = 0
    derivative = 0
    # twist.angular.z = 0
    # pub.publish(twist)
    return align_robotResponse(0)


rospy.init_node('Controller')
sub = rospy.Subscriber("odom",Odometry,get_rotation)
pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
pub_kp = rospy.Publisher("kp_gain",Float64,queue_size=1)

# pub_ki = rospy.Publisher("ki_gain",Float64,queue_size=1)
# pub_kd = rospy.Publisher("kd_gain",Float64,queue_size=1)
twist = Twist()
service =rospy.Service('control_robot',align_robot,align_now)
signal.signal(signal.SIGINT,exit_gracefully)
rospy.spin()
#need to call service in the mover code
