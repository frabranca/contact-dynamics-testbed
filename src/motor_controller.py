import time
import numpy as np
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
from motor_messages.motorlcm import motor_state
import lcm

class motor_controller:
    def __init__(self, can_port, motor_id, t, velocity, motor_type="AK80_9_V1p1", plot=False, channel = "MOTOR_STATE", communication=True):
        self.t = t
        self.velocity = velocity
        self.plot = plot
        self.channel = channel
        self.communication = communication

        self.lc = lcm.LCM()
        self.motor = CanMotorController(can_port, motor_id, motor_type=motor_type)
        self.motor.enable_motor()
        self.loop()

    def loop(self):
        start = time.time()
        mst = motor_state()
        i = 0
        time_ = []
        pos_ = []
        vel_ = []
        vel_f = []
        cur_ = []
        cur_f = []

        while (time.time() - start) < self.t:
            if (time.time() - start) >= 4:
                pos, vel, cur = self.motor.send_rad_command(0, 0, 0, 0, 0)
                self.motor.send_rad_command(0, 0, 0, 0, 0)
            else:
                pos, vel, cur = self.motor.send_deg_command(10, self.velocity, 0, 5, 0)
                #pos, vel, cur = self.motor.send_deg_command(0, 0, 0, 0, 0)
            # filter
            filter = 50
            filter_size = filter-1
            if i > filter_size+1:
                vel_filtered = sum(vel_[i-filter_size:i+1]) / (filter_size+1)
                cur_filtered = sum(cur_[i-filter_size:i+1]) / (filter_size+1)
            else:
                vel_filtered = 0
                cur_filtered = 0
            i+=1
            
            radius = 0.335
            # clockwise positive
            xmf = radius*np.cos(pos) + 0.125
            ymf = radius*np.sin(pos) + 0.100

            if self.communication:
                mst.motor_xmf = xmf
                mst.motor_ymf = ymf
                mst.motor_vel = vel
                mst.motor_cur = cur
                self.lc.publish(self.channel, mst.encode())

            if self.plot:
                time_.append(time.time()-start)
                pos_.append(pos)
                vel_.append(vel)
                vel_f.append(vel_filtered)
                cur_.append(cur)    
                cur_f.append(cur_filtered)

        print("Disabling Motors...")
        self.motor.disable_motor()

        if self.plot:
            cosine = np.cos(np.radians(pos_))
            plt.figure()
            plt.subplot(311)
            plt.plot(time_, cosine)
            plt.grid()
            plt.subplot(312)
            plt.plot(time_, vel_)
            plt.plot(time_, vel_f)
            plt.grid()
            plt.subplot(313)
            plt.plot(time_, cur_)
            plt.plot(time_, cur_f)
            plt.grid()
            
            print(abs(cosine[-1] - cosine[0]))
            plt.show()

if __name__=="__main__":
    can_port = 'can0'
    motor_id = 1
    motor_controller(can_port, motor_id, 10, 90, plot=True, communication=False)
    # motor = CanMotorController(can_port, motor_id, motor_type="AK80_9_V1p1")
    # motor.enable_motor()
    # motor.send_deg_command(10, 0, 0, 5, 0)
    # time.sleep(5)
    # motor.disable_motor()