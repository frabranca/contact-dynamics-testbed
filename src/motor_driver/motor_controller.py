import time
import numpy as np
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
from motor_messages.motorlcm import motor_state
import lcm

can_port = 'can0'
motor_id = 12
motor_01 = CanMotorController(can_port, motor_id, motor_type="AK80_9_V1p1")

class motor_controller:
    def __init__(self, can_port, motor_id, motor_type="AK80_9_V1p1", plot=False, channel = "MOTOR_STATE"):
        self.plot = plot
        self.channel = channel
        self.lc = lcm.LCM()
        self.motor = CanMotorController(can_port, motor_id, motor_type=motor_type)
        self.motor.enable_motor()
        self.loop()

    def loop(self):
        start = time.time()
        mst = motor_state()
        i = 0
        n = 10000
        time_ = []
        pos_ = []
        vel_ = []
        vel_f = []
        tau_ = []

        while i < n:
            if i > 8000:
                pos, vel, tau = self.motor.send_deg_command(0, 0, 0, 0, 0)
            else:
                pos, vel, tau = self.motor.send_deg_command(0, 20, 0, 5, 0)

            # filter
            filter = 20
            filter_size = filter-1
            if i > filter_size+1:
                vel_filtered = sum(vel_[i-filter_size:i+1]) / (filter_size+1)
                #vel_filtered = (vel_[i] + vel_[i-1] + vel_[i-2] + vel_[i-3] + vel_[i-4])/5
            else:
                vel_filtered = 0
            i+=1
            
            mst.motor_pos = pos
            mst.motor_vel = vel
            mst.motor_tau = tau
            self.lc.publish(self.channel, mst.encode())

            if self.plot == True:
                time_.append(time.time()-start)
                pos_.append(pos)
                vel_.append(vel)
                vel_f.append(vel_filtered)
                tau_.append(tau)    

        print("Disabling Motors...")
        self.motor.send_deg_command(0, 0, 0, 0, 0)
        self.motor.disable_motor()

        if self.plot:
            plt.subplot(211)
            plt.plot(time_, pos_)
            plt.subplot(212)
            plt.plot(time_, vel_)
            plt.plot(time_, vel_f)
            plt.show()

if __name__=="__main__":
    can_port = 'can0'
    motor_id = 12
    motor_controller(can_port, motor_id, plot=False)

    
