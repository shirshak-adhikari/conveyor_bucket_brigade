#!/usr/bin/env python

import rospy
from shirshak_bucket_brigade.srv import bucket,bucketResponse
from geometry_msgs.msg import Twist

def switch(value):
    global command
    pub.publish(command)
    # rospy.loginfo(rospy.get_time())
    rospy.sleep(2)
    # rospy.loginfo(rospy.get_time())
    return bucketResponse(0)


rospy.init_node('bucket_server')
service = rospy.Service('bucket_test',bucket,switch)
pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
command = Twist()
rospy.spin()
