#!/usr/bin/env python

import can, cantools
from std_msgs.msg import Float32

db = cantools.database.load_file('canSetup.kcd')
msg_turn = db.get_message_by_name('ARCommandMessage')
msg_move = db.get_message_by_name('logitechthrottle')
can_bus = can.interface.Bus(bustype = 'socketcan', channel = 'vcan0', bitrate = 500000)

val_turn = 0
val_move = 0

turn = msg_turn.encode({'CommandMode':5, 'Variable1':val_turn, 'Variable2':1})
move = msg_move.encode({'throttlesignal':val_move})

data_turn = can.Message(arbitration_id=msg_turn.frame_id, data=turn)
data_move = can.Message(arbitration_id=msg_move.frame_id, data=move)

task_turn = can_bus.send_periodic(data_turn, 1)
task_move = can_bus.send_periodic(data_move, 1)

cont = ''

while(cont.upper() != 'N'):
  try:
    val_turn = input('Turn: ')
  except:
    print('Invalid Input')
  try:
    val_move = input('Move: ')
  except:
    print('Invalid Input')
  
  if val_turn < -1.25:
    val_turn = -1.25
  elif val_turn > 1.25:
    val_turn = 1.25
  
  if val_move < 0:
    val_move = 0
  elif val_move > 12:
    val_move = 12
  
  turn = msg_turn.encode({'CommandMode':5, 'Variable1':val_turn, 'Variable2':1})
  move=msg_move.encode({'throttlesignal':val_move})

  data_turn = can.Message(arbitration_id=msg_turn.frame_id, data=turn)
  data_move = can.Message(arbitration_id=msg_move.frame_id, data=move)
  
  task_turn.modify_data(data_turn)
  task_move.modify_data(data_move)

  cont = str(raw_input('Continue (Y/n): '))

task_turn.stop()
task_move.stop()

turn = msg_turn.encode({'CommandMode':0, 'Variable1':0, 'Variable2':0})
data_turn = can.Message(arbitration_id=msg_turn.frame_id, data=turn)
can_bus.send(data_turn)

