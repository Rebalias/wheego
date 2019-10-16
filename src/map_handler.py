#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv
from geometry_msgs.msg import PoseStamped as PoseSt

xOffs = 0
yOffs = 0
init = False
pose2 = PoseCv()
goal2 = PoseSt()

def mapMask(map1):
  global xOffs
  global yOffs
  global init
  xOffs = map1.info.width
  yOffs = map1.info.height
  if not init:
    iPose = PoseCv()
    iPose.header.frame_id = 'map'
    iPose.pose.pose.position.x = xOffs/2
    iPose.pose.pose.position.y = yOffs/2
    iPose.pose.pose.orientation.w = 1.0
    iPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    pubIP.publish(iPose)
    init = True

def poseMask(pose1):
  global pose2
  pose2 = PoseCv()
  pose2.header.frame_id = 'map'
  pose2.pose = pose1.pose
  pubPD.publish(pose2)

def goalFn(goal1):
  global goal2
  goal2 = PoseSt()
  goal2.header.frame_id = 'map'
  goal2.pose = goal1.pose
  pubGD.publish(goal2)
  bcast()
  
def pathFn(foo):
  bcast()
  time.sleep(5.0)

def bcast():
  pubPN.publish(pose2)
  pubGN.publish(goal2)

rospy.init_node('map_handler')

pubPD = rospy.Publisher('pose_dis',PoseCv,latch=True,queue_size=5)
pubPN = rospy.Publisher('pose_nav',PoseCv,latch=True,queue_size=5)
pubGD = rospy.Publisher('goal_dis',PoseSt,latch=True,queue_size=5)
pubGN = rospy.Publisher('goal_nav',PoseSt,latch=True,queue_size=5)
pubIP = rospy.Publisher('initialpose',PoseCv,latch=True,queue_size=5)

subM = rospy.Subscriber('map',OccupancyGrid,mapMask)
subP = rospy.Subscriber('poseupdate',PoseCv,poseMask)
subG = rospy.Subscriber('move_base_simple/goal',PoseSt,goalFn)
subS = rospy.Subscriber('sPath',Path,pathFn)

rospy.spin()
