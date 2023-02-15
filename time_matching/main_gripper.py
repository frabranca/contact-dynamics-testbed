import lcm
from robot_messages.frankalcm import gripper_command
import time
import numpy as np
import matplotlib.pyplot as plt

""" - LCM messenger file to the motor_controller.py.
    - Used to debug the motor controller individually """

gripper_width = 0.02
gripper_speed = 10.0
gripper_force = 60.0
time.sleep(3)
lc = lcm.LCM()
gcm = gripper_command()
gcm.width = gripper_width
gcm.speed = gripper_speed
gcm.force = gripper_force
lc.publish("GRIPPER COMMAND", gcm.encode())
print("command sent")
