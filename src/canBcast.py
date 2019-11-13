#!/usr/bin/env python

import rospy, can, cantools
from std_msgs.msg import Float32

db = cantools.database.load_file('canSetup.kcd')
turnForm = db.get_message_by_name('ARCommandMessage')
moveForm = db.get_message_by_name('logitechthrottle')
canBus = can.interface.Bus(bustype = 'socketcan', channel = 'vcan0', bitrate = 500000)
canTwo = can.interface.Bus(bustype = 'socketcan', channel = 'vcan1', bitrate = 500000) #TODO these shouldn't be the same bitrate?
listener = can.BufferedListener()

turnVal = 0.0
rotSpeed = 0.1
moveVal = 0.0
targVel = 0.0

turnData= turnForm.encode({'CommandMode':5, 'Variable1':turnVal, 'Variable2':rotSpeed})
moveData= moveForm.encode({'throttlesignal':moveVal})

turnMsg = can.Message(arbitration_id = turnForm.frame_id, data = turnData)
moveMsg = can.Message(arbitration_id = moveForm.frame_id, data = moveData)

#turnTask = canBus.send_periodic(turnMsg, 0.1)
#moveTask = canBus.send_periodic(moveMsg, 0.1)

def turnFn(angle):
  turnVal = angle * 4 #scalar, previous prgm ranged -1.25 to 1.25
  if turnVal < -1.25:
    turnVal = 1.25
  elif turnVal > 1.25:
    turnVal = 1.25
  turnData = turnForm.encode({'CommandMode':5, 'Variable1':turnVal, 'Variable2':rotSpeed})
  turnMsg = can.Message(arbitration_id=turnForm.frame_id, data=turnData)
  #turnTask.modify_data(turnMsg)
  canBus.send(turnMsg)

def moveFn(comm):
  global targVel
  targVel = comm

rospy.init_node('canBcast')

subAng = rospy.Subscriber('steerAngle', Float32, turnFn)
subMov = rospy.Subscriber('moveSpeed', Float32, moveFn)

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

#turnTask.stop()
#moveTask.stop()

turnData = turnForm.encode({'CommandMode':0,'Variable1':0,'Variable2':0})
turnMsg = can.Message(arbitration_id=turnForm.frame_id, data=turnData)
canBus.send(data_turn)


