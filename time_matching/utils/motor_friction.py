import time
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
import numpy as np

time_ = []
pos_ = []
vel_ = []
tau_ = []
vel_filter = []

cf = 0.095 # v = 20
# cf = 0.01

def tau_friction(v):
    return cf*np.arctan(100*v)

motor = CanMotorController('can0', 3, motor_type="AK80_6_V1p1")
motor.enable_motor()
start = time.time()
i=0

pos_des = 0
vel_des = 20
Kp = 0
Kd = 5
tau_des = 0.15
vel_meas = 0

while (time.time()-start) < 10.:
    if (time.time()-start) < 5.:
        torque = tau_friction(vel_meas) + tau_des
        pos_meas, vel_meas, tau_meas = motor.send_deg_command(pos_des, vel_des, Kp, Kd, torque)
    # pos, vel, tau = motor.send_deg_command(0, 0, 0, 0, 0)
    else:
        torque = tau_friction(vel_meas)
        pos_meas, vel_meas, tau_meas = motor.send_deg_command(0, 0, 0, 0, torque)

    time_.append(time.time()-start)
    pos_.append(pos_meas)
    vel_.append(vel_meas)
    tau_.append(tau_meas)

    filter = 100
    filter_size = filter-1
    if i > filter_size+1:
        vel_filtered = sum(vel_[i-filter_size:i+1]) / (filter_size+1)
    else:
        vel_filtered = 0

    vel_filter.append(vel_filtered)
    i+=1



motor.disable_motor()

plt.plot(time_, vel_)
plt.plot(time_, vel_filter)

plt.show()
