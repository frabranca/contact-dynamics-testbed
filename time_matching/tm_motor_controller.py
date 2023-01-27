import time
import lcm
import numpy as np
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
from robot_messages.motorlcm import motor_command

class motor_controller:
    def __init__(self, can_port, motor_id, t, velocity, motor_type="AK80_9_V1p1", channel = "MOTOR_COMMAND"):
        self.t = t
        self.velocity = velocity
        self.channel = channel

        self.lc = lcm.LCM()
        self.lc.subscribe(self.channel, self.motor_handler)
        self.motor = CanMotorController(can_port, motor_id, motor_type=motor_type)
        self.motor.enable_motor()

        self.lc.handle()
        self.loop()

    def loop(self):
        start = time.time()

        while (time.time() - start) < self.t:
            if (time.time() - start) >= 7.:
                pos, vel, cur = self.motor.send_deg_command(0, 0, 0, 0, 0)
                # self.motor.send_rad_command(0, 0, 0, 0, 0)
            else:
                pos, vel, cur = self.motor.send_deg_command(0, self.velocity, 0, 5, 2)
                #pos, vel, cur = self.motor.send_deg_command(0, 0, 0, 0, 0)        
        
        print("Disabling Motors...")
        self.motor.disable_motor()
    
    def motor_handler(self, channel, data):
        mcm = motor_command.decode(data)
        self.motor_enable = mcm.motor_enable

if __name__=="__main__":
    can_port = 'can0'
    motor_id = 1
    motor_controller(can_port, motor_id, 14, 60)