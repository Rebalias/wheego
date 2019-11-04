#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import can4python as can

def callback(data):
  print('foo')

rospy.init_node('canBcast')

sub = rospy.Subscriber('steerAngle', Float32, callback)

rospy.spin()

