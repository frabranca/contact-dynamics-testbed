import time
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
import numpy as np

time_ = []
pos_ = []
vel_ = []
tau_ = []

def friction(v):
    return np.atan(100*v)

motor = CanMotorController('can0', 1, motor_type="AK80_9_V1p1")
motor.enable_motor()
start = time.time()
while (time.time()-start) < 10.:
    pos, vel, tau = motor.send_deg_command(0, 5, 0, 1, 0.1)

    time_.append(time.time()-start)
    pos_.append(pos)
    vel_.append(vel)
    tau_.append(tau)

motor.disable_motor()

plt.plot(time_, vel_)
plt.show()