#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_motor_command_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

turn_command = mbot_motor_command_t()
turn_command.trans_v = 0.0
turn_command.angular_v = 3.14159/2

ramp_command = mbot_motor_command_t()
ramp_command.trans_v = 0.25
ramp_command.angular_v = 0.0

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.5
drive_command.angular_v = 0.0

lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
sleep(4.0)
lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(4)
lc.publish("MBOT_MOTOR_COMMAND",ramp_command.encode())
sleep(0.75)
lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
sleep(0.75)
