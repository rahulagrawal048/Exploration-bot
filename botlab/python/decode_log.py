import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("lcmtypes")
import lcm
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

slam_data = np.empty((0,4), dtype=float)
true_data = np.empty((0,4), dtype=float)
init = 0
for event in log:
    if event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        slam_data = np.append(slam_data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

init = 0
for event in log:
    if event.channel == "TRUE_POSE":
        msg2 = pose_xyt_t.decode(event.data)
        if init==0:
            start_utime = msg2.utime
            init = 1
        true_data = np.append(true_data, np.array([[ \
            (msg2.utime-start_utime)/1.0E6, \
            msg2.x, \
            msg2.y, \
            msg2.theta
            ]]), axis=0)





#plt.plot(data[:,1], data[:,2], 'r')
#plt.show()