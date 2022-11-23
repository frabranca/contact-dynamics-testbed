import lcm
import time
from math import *

from exlcm import command

# START LCM
lc = lcm.LCM()
msg = command()

#DEFINE MESSAGE
msg.q_goal = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]

#PUBLISH ON CHANNEL
lc.publish("COMMAND", msg.encode())

