#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv

offs = 0

def mapMask(map1):
  map2 = OccupancyGrid()
  map2.header = map1.header
  map2.info = map1.info
  global offs
  offs = map1.info.width
  map2.info.origin.position.x = 0
  map2.info.origin.position.y = 0
  map2.data = [0 if x==-1 else x for x in map1.data]
  pubM.publish(map2)


rospy.init_node('map_mask')
subM = rospy.Subscriber('map',OccupancyGrid,mapMask)
pubM = rospy.Publisher('map_nav',OccupancyGrid,latch=True,queue_size=2)
rospy.spin()
