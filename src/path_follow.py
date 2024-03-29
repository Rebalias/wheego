#!/usr/bin/env python

import rospy
import numpy as np
from math import pi
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseCv

path = np.array([])
L = 2.0   #Length from rear axle to front axle
ld = 7.5  #Look ahead distance

def poseFn(pose):
  if path.shape[0] > 0:
    x = pose.pose.pose.position.x
    y = pose.pose.pose.position.y
    #Vectorizing orientation
    theta = 2 * np.arcsin(pose.pose.pose.orientation.z)
    loc = np.array([x,y])
    p = [np.cos(theta + pi/2), np.sin(theta + pi/2)]
    #Find furthest path point within lookahead distance
    #ang = np.array([(np.arccos(np.dot(path[i], path[i+1]) / (np.linalg.norm(path[i]) * np.linalg.norm(path[i+1])))) for i in range(path.shape[0])])
    dist = np.linalg.norm(path - loc, axis=1)
    ind = np.flatnonzero((dist[:-1]<ld) & (dist[1:]>ld))
    if ind.size == 0:
      if np.max(dist<ld):
        i = dist.size-1
      else:
        i = 0
    else:
      i = ind[0]
    g = path[i]
    t = g - loc
    d = np.linalg.norm(t) #maybe decrease speed if d << ld
    el = np.dot(t, p) #Lateral distance
    steer = np.arctan(2*L*el/(d**2))
    pub.publish(steer)

#Converts sPath to numpy array
def pathFn(data):
  global path
  if len(data.poses) > 0:
    cx = [foo.pose.position.x for foo in data.poses]
    cy = [foo.pose.position.y for foo in data.poses]
    path = np.matrix.transpose(np.array([cx, cy]))
    print(path)

rospy.init_node('path_follow')

pub = rospy.Publisher('steerAngle', Float32, latch=True, queue_size=5)
subPath = rospy.Subscriber('sPath', Path, pathFn)
subPose = rospy.Subscriber('poseupdate', PoseCv, poseFn)

rospy.spin()
