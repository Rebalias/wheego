#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
  print(msg.width)

rospy.init_node('lidar_subscriber')
sub = rospy.Subscriber('velodyne_points',PointCloud2,callback)
rospy.spin()
