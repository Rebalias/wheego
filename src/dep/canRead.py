#!/usr/bin/env python

import can

canBus = can.Bus(interface='socketcan', channel='can1')

for msg in canBus:
  if msg.arbitration_id == 0x200:
    print(msg)

