import lcm
from motor_messages.motorlcm import motor_command
import time
import numpy as np
import matplotlib.pyplot as plt

time.sleep(3)
lc = lcm.LCM()
mcm = motor_command()
mcm.motor_enable = True
lc.publish("MOTOR COMMAND", mcm.encode())
print("command sent")
motor_moved = True