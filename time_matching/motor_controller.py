import time
import lcm
import numpy as np
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
from motor_messages.motorlcm import motor_command

class motor_controller:
    def __init__(self, can_port, motor_id, t, velocity, motor_type="AK80_9_V1p1", channel = "MOTOR COMMAND"):
        self.t = t
        self.velocity = velocity
        self.channel = channel

        self.t_save   = []
        self.pos_save = []
        self.vel_save = []
        self.cur_save = []

        self.lc = lcm.LCM()
        self.lc.subscribe(self.channel, self.motor_handler)
        self.motor = CanMotorController(can_port, motor_id, motor_type=motor_type)
        self.motor.enable_motor()

        self.lc.handle()
        self.loop()
        self.show_plot()

        self.vel_save = np.array(self.vel_save)
        self.write_data()

    def loop(self):
        start = time.time()

        while (time.time() - start) < self.t:
            if (time.time() - start) >= 7.:
                pos, vel, cur = self.motor.send_deg_command(0, 0, 0, 0, 0)
                # self.motor.send_rad_command(0, 0, 0, 0, 0)
            else:
                pos, vel, cur = self.motor.send_deg_command(0, self.velocity, 0, 5, 2)
                #pos, vel, cur = self.motor.send_deg_command(0, 0, 0, 0, 0)        
            
            self.t_save.append(time.time()-start)
            self.pos_save.append(pos)
            self.vel_save.append(vel)
            self.cur_save.append(cur)

            
        print("Disabling Motors...")
        self.motor.disable_motor()
    
    def motor_handler(self, channel, data):
        mcm = motor_command.decode(data)
        self.motor_enable = mcm.motor_enable
    
    def show_plot(self):
        plt.figure()
        plt.plot(self.t_save, self.vel_save)
        plt.xlabel("time [s]")
        plt.ylabel("velocity [deg/s]")
        plt.grid()
        plt.savefig("satellite_velocity.png")
        plt.show()
    
    def write_data(self):
        output = open("satellite_velocity", "w")
        output.truncate()
        for i in range(len(self.vel_save)):
            output.write(str(self.t_save[i]) + ' ' + ' ' + str(self.vel_save[i]) + '\n')
        output.close()

if __name__=="__main__":
    can_port = 'can0'
    motor_id = 1
    motor_controller(can_port, motor_id, 14, 60)
