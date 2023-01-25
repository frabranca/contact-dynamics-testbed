import lcm
import time
import matplotlib.pyplot as plt
import numpy as np
from robot_messages.frankalcm import robot_command, gripper_command, robot_state
from robot_messages.motorlcm import motor_command

class Controller:
    def __init__(self, rcm_channel, gcm_channel, mcm_channel, rst_channel, save_data = True):
        self.rcm_channel = rcm_channel
        self.gcm_channel = gcm_channel
        self.mcm_channel = mcm_channel
        self.rst_channel = rst_channel

        self.save_data = save_data

        # robot states
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0

	# robot save lists
        self.time_save = []
        self.q_save = []
        self.q_d_save = []
        self.dq_save = []
        self.dq_d_save = []
        self.ddq_d_save = []
        self.tau_J_save = []
        self.tau_J_d_save = []
        self.dtau_J_save = []

        self.lc = lcm.LCM()
        self.robot_sub = self.lc.subscribe(self.rst_channel, self.robot_handler)

        self.control_loop()

        self.lc.unsubscribe(self.robot_sub)
        self.plot()
        
    def message(self, string):
        print("controller.py: " + string)
    
    def robot_handler(self, channel, data):
        rst = robot_state.decode(data)
        self.q             = rst.q
        self.q_d           = rst.q_d
        self.dq            = rst.dq
        self.dq_d          = rst.dq_d
        self.ddq_d         = rst.ddq_d
        self.tau_J         = rst.tau_J
        self.tau_J_d       = rst.tau_J_d
        self.dtau_J        = rst.dtau_J
    
    def plot(self):
        plt.plot(self.time_save, self.q_save)
        plt.show()
        
    def control_loop(self):
        start = time.time()
        gcm_sent = False
        rcm_sent = False
        mcm_sent = False

        satellite_time = 7.6541
        motor_time = 1.
        robot_time = 1. + satellite_time - 1.#1.35439
        gripper_time = 1. + satellite_time - 0.5# - 0.6523
        
        #satellite_time = 7.6541
        #motor_time = 1.
        #robot_time = 1. + satellite_time - 0.6#1.35439
        #gripper_time = 1. + satellite_time - 0.5# - 0.6523
        while (time.time()-start) <= 15.:

            gcm = gripper_command()
            rcm = robot_command()
            mcm = motor_command()

            if (time.time()-start) >= gripper_time and gcm_sent == False:
                gcm.gripper_enable = True
                self.lc.publish(self.gcm_channel, gcm.encode())

                gcm_sent = True
            
            if (time.time()-start) >= robot_time and rcm_sent == False:
                rcm.robot_enable = True
                self.lc.publish(self.rcm_channel, rcm.encode())
                
                rcm_sent = True
                
            if rcm_sent == True:
                self.lc.handle()
                if self.save_data:
                    self.time_save.append(time.time()-start)
                    self.q_save.append(self.q)
            
            if (time.time()-start) >= motor_time and mcm_sent == False:
                mcm.motor_enable = True
                self.lc.publish(self.mcm_channel, mcm.encode())

                mcm_sent = True
        
            #if self.save_data:
                #self.time_save.append(time.time()-start)
                #self.q_save.append(self.q)
    
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", 
                            "GRIPPER COMMAND", 
                            "MOTOR_COMMAND",
                            "ROBOT STATE")
