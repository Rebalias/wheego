#!/usr/bin/env python
import rospy
import time
import numpy as np
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv
from geometry_msgs.msg import PoseStamped as PoseSt

xOffs = 0
yOffs = 0
init = False
pose2 = PoseCv()
goal1 = PoseSt()
goal2 = PoseSt()
dist = 5

#Publish hector starting position to center of map
#Neccesary, because map is not centered on 0,0 for hybrid astar
def mapMask(map1):
  global xOffs
  global yOffs
  global init
  xOffs = map1.info.width
  yOffs = map1.info.height
  res = round(map1.info.resolution, 5)
  if not init:
    iPose = PoseCv()
    iPose.header.frame_id = 'map'
    iPose.pose.pose.position.x = xOffs*res/2
    iPose.pose.pose.position.y = yOffs*res/2
    iPose.pose.pose.orientation.w = 1.0
    iPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]  #standard covariances generated by rviz
    pubIP.publish(iPose)
    init = True

#Saves latest poseupdate from hector
#Add goal-to-publish determination
def poseMask(pose1):
  global pose2
  global goal2
  global backTarget
  pose2 = PoseCv()
  pose2.header.frame_id = 'map'
  pose2.pose = pose1.pose
  #pubPD.publish(pose2)
  
  p = np.array([pose1.pose.pose.position.x, pose1.pose.pose.position.y])
  g = np.array([goal1.pose.position.x, goal1.pose.position.y])
  d = np.linalg.norm(p-g)
  if d < 10:
    theta = 2 * np.arcsin(goal1.pose.orientation.z)
    goal2.pose.position.x = goal1.pose.position.x #+ dist * np.cos(theta)
    goal2.pose.position.y = goal1.pose.position.y #+ dist * np.sin(theta)
  else:
    theta = 2 * np.arcsin(goal1.pose.orientation.z)
    goal2.pose.position.x = goal1.pose.position.x - dist * np.cos(theta)
    goal2.pose.position.y = goal1.pose.position.y - dist * np.sin(theta)

#Saves latest goal from rviz, calls bcast
def goalFn(goalD):
  global goal1
  global goal2
  goal1 = goalD
  goal2 = PoseSt()
  goal2.header.frame_id = 'map'
  goal2.pose = deepcopy(goal1.pose)
  theta = 2 * np.arcsin(goal1.pose.orientation.z)
  goal2.pose.position.x -= dist * np.cos(theta)
  goal2.pose.position.y -= dist * np.sin(theta)
  
  #pubGD.publish(goal2)
  bcast()

#Waits 5 seconds after sPath is published, then calls bcast
def pathFn(foo):
  time.sleep(1.0)
  bcast()

#Broadcasts start and end poses to hybrid A*
def bcast():
  #pubPN.publish(pose2)
  pubGN.publish(goal2)

rospy.init_node('map_handler')

#pubPD = rospy.Publisher('pose_dis',PoseCv,latch=True,queue_size=5)
#pubPN = rospy.Publisher('pose_nav',PoseCv,latch=True,queue_size=5)
#pubGD = rospy.Publisher('goal_dis',PoseSt,latch=True,queue_size=5)
pubGN = rospy.Publisher('goal_nav',PoseSt,latch=True,queue_size=5)
pubIP = rospy.Publisher('initialpose',PoseCv,latch=True,queue_size=5)

subM = rospy.Subscriber('map',OccupancyGrid,mapMask)
subP = rospy.Subscriber('poseupdate',PoseCv,poseMask)
subG = rospy.Subscriber('move_base_simple/goal',PoseSt,goalFn)
subS = rospy.Subscriber('move_base/NavfnROS/plan',Path,pathFn)

rospy.spin()
