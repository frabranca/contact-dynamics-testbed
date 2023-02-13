import time
import lcm
import numpy as np
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
from motor_messages.motorlcm import motor_command, motor_state

""" 

motor_controller.py: this file works as a communication interface to the motor.

    - receives enabling message from controller.py from channel "MOTOR COMMAND"
    - spins the motor at constant speed for 5 seconds with torque commands
    - switches to friction compensation for the remaining time

"""

class motor_controller:
    def __init__(self, can_port, motor_id, cf, velocity, motor_type="AK80_6_V1p1", mcm_channel = "MOTOR COMMAND", mst_channel = "MOTOR STATE"):

        self.cf          = cf          # coulomb friction coefficient
        self.velocity    = velocity    # satellite rotational velocity
        self.mcm_channel = mcm_channel
        self.mst_channel = mst_channel

        self.t_save          = []
        self.pos_save        = []
        self.vel_save        = []
        self.vel_filter_save = []
        self.tau_save        = []

        self.lc = lcm.LCM()
        self.lc.subscribe(self.mcm_channel, self.motor_handler)
        self.motor = CanMotorController(can_port, motor_id, motor_type=motor_type)
        _, _, _ = self.motor.set_zero_position()
        _, _, _ = self.motor.enable_motor()

        self.lc.handle()
        self.loop()
        self.show_plot()

        self.write_data()
    
    def friction_compensation(self, v):
        return self.cf*np.arctan(100*v)

    def loop(self):
        start = time.time()
        mst = motor_state()

        # control gains
        Kp = 0
        Kd = 5

        # desired position, velocity and torque
        pos_des = 0
        vel_des = self.velocity
        tau_des = 0.15

        # measured position, velocity, torque
        pos_meas = 0
        vel_meas = 0
        tau_meas = 0

        # index for filtered measurements
        i = 0

        while (time.time() - start) < 10:
            if (time.time() - start) >= 5.:
                torque = self.friction_compensation(vel_meas)
                pos_meas, vel_meas, tau_meas = self.motor.send_deg_command(0, 0, 0, 0, torque)
            else:
                torque = self.friction_compensation(vel_meas) + tau_des
                pos_meas, vel_meas, tau_meas = self.motor.send_deg_command(pos_des, vel_des, Kp, Kd, torque)
            
            mst.position = pos_meas
            self.lc.publish(self.mst_channel, mst.encode())

            filter = 100
            filter_size = filter-1
            if i > filter_size+1:
                vel_filtered = sum(self.vel_save[i-filter_size:i+1]) / (filter_size+1)
            else:
                vel_filtered = 0

            self.vel_filter_save.append(vel_filtered)
            i+=1

            self.t_save.append(time.time()-start)
            self.pos_save.append(pos_meas)
            self.vel_save.append(vel_meas)
            self.tau_save.append(tau_meas)

            
        print("Disabling Motors...")
        self.motor.disable_motor()
    
    def motor_handler(self, channel, data):
        mcm = motor_command.decode(data)
        self.motor_enable = mcm.motor_enable
    
    def show_plot(self):
        plt.figure()
        plt.title("cf = " + str(self.cf))
        plt.plot(self.t_save, self.vel_save)
        plt.plot(self.t_save, self.vel_filter_save)
        plt.xlabel("time [s]")
        plt.ylabel("velocity [deg/s]")
        plt.legend(["measured", "filtered"], loc='best')
        plt.grid()
        plt.savefig("satellite_velocity.png")
        plt.show()
    
    def write_data(self):
        output = open("satellite_velocity", "w")
        output.truncate()
        output.write("time" + " " + "velocity" + " " + "velocity_filtered" + "\n")
        for i in range(len(self.vel_save)):
            output.write(str(self.t_save[i]) + ' ' + ' ' + str(self.vel_save[i]) + ' ' + str(self.vel_filter_save[i]) + '\n')
        output.close()

if __name__=="__main__":
    can_port = 'can0'
    motor_id = 3
    motor_controller(can_port, motor_id, 0.08, 20)
