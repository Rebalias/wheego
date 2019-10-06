#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def callback(map1):
  map2 = OccupancyGrid()
  map2.header = map1.header
  map2.info = map1.info
  map2.data = [0 if x==-1 else x for x in map1.data]
  pub.publish(map2)

rospy.init_node('map_mask')
sub = rospy.Subscriber('map',OccupancyGrid,callback)
pub = rospy.Publisher('map2',OccupancyGrid,latch=True)
rospy.spin()
