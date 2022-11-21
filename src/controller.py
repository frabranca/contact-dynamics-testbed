#from curses import KEY_DC
import lcm
import time
from math import *
from exlcm import state, command
import numpy as np

# This code implements gravity compensation on the Franka robot.
# The torque / joint angles / velocities are read from the cpp file torque_control.cpp 
# The torque commands are sent to the cpp file torque_control.cpp

# reads robot states from LCM channel
def my_handler(channel, data):
    st = state.decode(data)
    global q, dq, dq_d, tau_J, tau_J_d, dtau_J, width, max_width, is_grasped
    q          = st.q
    dq         = st.dq
    dq_d       = st.dq_d
    tau_J      = st.tau_J
    tau_J_d    = st.tau_J_d
    dtau_J     = st.dtau_J
    width      = st.width
    max_width  = st.max_width
    is_grasped = st.is_grasped


# GRIPPER FUNCTION
# def gripper_msg(command):
#     threading.Timer(5.0, gripper_msg).start()
#     command.width = 0.02

# DEFINE MESSAGE / CONTROLLER
# timeout = 0.001

# START LCM
lc = lcm.LCM()
subscription = lc.subscribe("STATE", my_handler)

try:
    while True:
        lc.handle()
        cmd = command()
        cmd.tau_J_d = tau_J_d
        lc.publish("COMMAND", cmd.encode())

except KeyboardInterrupt:
    pass
        
lc.unsubscribe(subscription)
