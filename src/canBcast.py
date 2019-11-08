#!/usr/bin/env python

import rospy, can, cantools
from std_msgs.msg import Float32

db = cantools.database.load_file('canSetup.kcd')
msg_turn = db.get_message_by_name('ARCommandMessage')
msg_move = db.get_message_by_name('logitechthrottle')
can_bus = can.interface.Bus(bustype = 'socketcan', channel = 'vcan0', bitrate = 500000)
can_two = can.interface.Bus(bustype = 'socketcan', channel = 'vcan1', bitrate = 500000) #TODO these shouldn't be the same bitrate?
listener = can.BufferedListener()

targ_vel = 0

def turnFn(angle):
  val = angle * 4 #scalar, previous prgm ranged -1.25 to 1.25
  if val < -1.25:
    val = 1.25
  elif val > 1.25:
    val = 1.25
  turn = msg_turn.encode({'CommandMode':5,'Variable1':val,'Variable2':1})
  data_turn = can.Message(arbitration_id=msg_turn.frame_id, data=turn)
  can_bus.send_periodic(data_turn, 0.2) #TODO set send interval

def moveFn(comm):
  global targ_vel
  targ_vel = comm

rospy.init_node('canBcast')

subAng = rospy.Subscriber('steerAngle', Float32, turnFn)
subMov = rospy.Subscriber('TODO', Float32, moveFn)

while(True):
  msg = listener.get_message() #TODO Not sure where to specify which bus?
  if msg is not None:
    #This won't work until we have the car CAN as well
    data = db.decode(msg.arbitration_id, msg.data)
    cur_vel = data=['velocity'] # * scalar?
    acc = (targ_vel - cur_vel) # * scalar, previous prgm ranged 0 to 12
    if acc < 0:
      acc = 0.0
    elif acc > 12:
      acc = 12.0
    move=msg_move.encode({'throttlesignal':acc})
    data_move = can.Message(arbitration_id=msg_move.frame_id, data=move)
    can_bus.send_periodic(data_move, 0.2) #TODO set send interval

