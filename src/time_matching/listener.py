import lcm
from robot_messages.frankalcm import robot_state, robot_command, gripper_command
from robot_messages.motorlcm import motor_command
import time
import matplotlib.pyplot as plt
import numpy as np

q = []
dq = []
tau_J = []
t = []
def robot_handler(channel, data):
    rst = robot_state.decode(data)
    q.append(rst.q)
    dq.append(rst.dq)
    tau_J.append(rst.tau_J)

lc = lcm.LCM()
robot_sub   = lc.subscribe("ROBOT STATE", robot_handler)
start = time.time()
while True:
    if (time.time()-start)<10.:
        t.append(time.time()-start)
        lc.handle()
    else:
        break

q = np.array(q)
dq = np.array(dq)
tau_J = np.array(tau_J)
plt.plot(t,q[:,0])
plt.plot(t,dq[:,0])
plt.plot(t,tau_J[:,0])
plt.show()
