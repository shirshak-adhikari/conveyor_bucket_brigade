#!/bin/sh
sudo ROS_NAMESPACE=tbota roslaunch shirshak_bucket_brigade Bucket_Brigade_launch.launch
sudo ROS_NAMESPACE=tbotb roslaunch shirshak_bucket_brigade Bucket_Brigade_launch.launch
sudo ROS_NAMESPACE=tbotc roslaunch shirshak_bucket_brigade Bucket_Brigade_launch.launch
sudo ROS_NAMESPACE=tbotd roslaunch shirshak_bucket_brigade Bucket_Brigade_launch.launch
sudo ROS_NAMESPACE=tbote roslaunch shirshak_bucket_brigade Bucket_Brigade_launch.launch

