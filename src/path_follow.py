#!/usr/bin/env python

import rospy
import numpy as np
from math import pi
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv

pathF = Path()
L = 1   #Length from rear axle to front axle
ld = 10  #Look ahead distance

def poseFn(pose):
  x = pose.pose.pose.position.x
  y = pose.pose.pose.position.y
  #Vectorizing orientation
  theta = 2 * np.arcsin(pose.pose.pose.orientation.z)
  loc = np.array([x,y])
  px = np.cos(theta + pi/2)
  py = np.sin(theta + pi/2)
  #Turn pathF.poses[].pose.position into path[]
  pathx = [foo.pose.position.x for foo in pathF.poses]
  pathy = [foo.pose.position.y for foo in pathF.poses]
  path = np.matrix.transpose(np.array([pathx, pathy]))
  #needs better setup, maybe find closest position first?
  g = path[0]
  i = 1
  d = np.linalg.norm(loc - path[i])
  while d < ld and i <= path.shape[0]:
    g = path[i]
    i += 1
    d = np.linalg.norm(loc - path[i]) #Error if at end of path?
  #maybe decrease speed if d < ld
  tx = g[0] - loc[0]
  ty = g[1] - loc[1]
  el = tx*px + ty*py #Lateral distance
  steer = Float32()
  steer.data = np.arctan(2*L*el/(ld*ld))
  print(steer.data)
  #pub.publish(steer)

def pathFn(path):
  global pathF
  if len(path.poses) > 0:
    pathF = path

rospy.init_node('path_follow')

pub = rospy.Publisher('steerAngle',Float32,latch=True,queue_size=5)

subPath = rospy.Subscriber('sPath',Path,pathFn)
subPose = rospy.Subscriber('poseupdate',PoseCv,poseFn)

rospy.spin()
