#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

roll = pitch = yaw = 0.0

def get_rotation_now(msg):
    global roll, pitch, yaw
    orientation_q= msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)


def main():
    pub = rospy.Publisher('yaw', Float64, queue_size=10)
    pub1 = rospy.Publisher('setpoint',Float64,queue_size=10)
    sub = rospy.Subscriber("odom",Odometry,get_rotation_now)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(round(yaw,3))
        pub1.publish(1)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('plot_topic')
    main()

    rospy.spin()
