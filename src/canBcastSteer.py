#!/usr/bin/env python

import rospy, can, cantools
from std_msgs.msg import Float32, Bool

pkgdir = rospy.get_param('/can_bcast/pkgdir', '/home/ben/catkin_ws/src/wheego')
sim = rospy.get_param('/can_bcast/sim', True)

if sim:
  ch = 'vcan0'
else:
  ch = 'can0'

db = cantools.database.load_file(pkgdir+'/src/canSetup.kcd')
turnForm = db.get_message_by_name('ARCommandMessage')
moveForm = db.get_message_by_name('logitechthrottle')
canBus = can.interface.Bus(bustype = 'socketcan', channel = ch, bitrate = 500000)
#canTwo = can.interface.Bus(bustype = 'socketcan', channel = 'vcan1', bitrate = 500000) #TODO these shouldn't be the same bitrate?
#listener = can.BufferedListener()

turnVal = 0.0
rotSpeed = 0.2
moveVal = 0.0
targVel = 0.0
oldAng = 0.0
bcast = True
diff = 0.0

def turnFn(angle):
  global oldAng
  global diff
  turnVal = min(max(angle.data*-4,-1.25),1.25)
  rotSpeed = min(max(abs(angle.data*2),0.1),0.5)
  if(bcast):
    turnData = turnForm.encode({'CommandMode':5, 'Variable1':turnVal, 'Variable2':rotSpeed})
  else:
    turnData = turnForm.encode({'CommandMode':2, 'Variable1':0, 'Variable2':0})
  turnMsg = can.Message(arbitration_id=turnForm.frame_id, data=turnData)
  #turnTask.modify_data(turnMsg)
  canBus.send(turnMsg)

def moveFn(comm):
  global targVel
  targVel = comm
  
def modeFn(data):
  global bcast
  bcast = data.data

rospy.init_node('can_bcast')

subAng = rospy.Subscriber('steerAngle', Float32, turnFn)
subMov = rospy.Subscriber('moveSpeed', Float32, moveFn)
subMod = rospy.Subscriber('canState', Bool, modeFn)

"""
for msg in canTwo:
  data = db.decode(msg.arbitration_id, msg.data)
  if 'velocity' in data:
    curVel = data['velocity'] # * scalar?
    acc = int((targVel - curVel))  # * scalar?
    if acc < 0:
      acc = 0
    elif acc > 12:
      acc = 12
    moveData = moveForm.encode({'throttlesignal':acc})
    moveMsg = can.Message(arbitration_id = moveForm.frame_id, data = moveData)
    #moveTask.modify_data(moveMsg)
    canBus.send(moveMsg)
"""
#turnTask.stop()
#moveTask.stop()

rospy.spin()
turnData = turnForm.encode({'CommandMode':0,'Variable1':0,'Variable2':0})
turnMsg = can.Message(arbitration_id=turnForm.frame_id, data=turnData)
canBus.send(data_turn)


