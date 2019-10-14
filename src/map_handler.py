#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv
from geometry_msgs.msg import PoseStamped as PoseSt

xOffs = 0
yOffs = 0
map2 = OccupancyGrid()
pose2 = PoseCv()

"""
def mapMask(map1):
  global map2
  global xOffs
  global yOffs
  map2 = OccupancyGrid()
  map2.header = map1.header
  map2.info = map1.info
  xOffs = map1.info.width
  yOffs = map1.info.height
  map2.info.origin.position.x = 0
  map2.info.origin.position.y = 0
  map2.data = [0 for x in map1.data]
  #[0 if x==-1 else x for x in map1.data]
  pubMD.publish(map2)
"""

def poseMask(pose1):
  global pose2
  pose2 = PoseCv()
  pose2 = pose1
  pose2.pose.pose.position.x += xOffs/2
  pose2.pose.pose.position.y += yOffs/2
  pubPD.publish(pose2)

#Will hA* fail if a new map is broadcast?

#Broadcast new map and new start pose when hA* produces path or on manual command (recieving goal?)
def bcast(foo):
  #pubMN.publish(map2)
  #time.sleep(0.5)
  pubPN.publish(pose2)

rospy.init_node('map_handler')
subM = rospy.Subscriber('map',OccupancyGrid,mapMask)
subP = rospy.Subscriber('poseupdate',PoseCv,poseMask)
subG = rospy.Subscriber('move_base_simple/goal',PoseSt,bcast)
subS = rospy.Subscriber('sPath',Path,bcast)

pubMD = rospy.Publisher('map_dis',OccupancyGrid,latch=True,queue_size=2)
pubMN = rospy.Publisher('map_nav',OccupancyGrid,latch=True,queue_size=2)
pubPD = rospy.Publisher('pose_dis',PoseCv,latch=True,queue_size=5)
pubPN = rospy.Publisher('pose_nav',PoseCv,latch=True,queue_size=5)

rospy.spin()
