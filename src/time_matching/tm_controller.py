import lcm
import time
import matplotlib.pyplot as plt
import numpy as np
from robot_messages.frankalcm import robot_command, gripper_command, motor_command

class Controller:
    def __init__(self, rst_channel, rcm_channel, gst_channel, gcm_channel, mst_channel, save_output=False, plot_data=False):
        self.control_loop()
        # initiate state variables as zeros

        # robot states

        # gripper states
        
        # motor states
        
        # useful booleans
        
        # lists to save output
        
        # define lcm channels
        
        # subscribe to channels
        
        # calculate trajectory
        
        # actions
        
    def message(self, string):
        print("controller.py: " + string)

    def control_loop(self):
        start = time.time()
        gcm_sent = False
        rcm_sent = False
        mcm_sent = False

        satellite_time = 7.6541
        motor_time = 1.
        robot_time = 1. + satellite_time - 0.73798
        gripper_time = 1. + satellite_time - 1.460984

        while (time.time()-start) <= 15.:

            if (time.time()-start) >= gripper_time and gcm_sent == False:
                gcm = gripper_command()
                gcm.gripper_enable = True
                self.lc.publish(self.gcm_channel, gcm.encode())

                gcm_sent = True
            
            if (time.time()-start) >= robot_time and rcm_sent == False:
                rcm = robot_command()
                rcm.robot_enable = True
                self.lc.publish(self.rcm_channel, rcm.encode())
                
                rcm_sent = True
            
            if (time.time-start) >= motor_time and mcm_sent == False:
                mcm = motor_command()
                mcm.motor_enable = True
                self.lc.publish(self.mcm_channel, mcm.encode())

                mcm_sent = True
    
if __name__ == "__main__":
    controller = Controller("ROBOT STATE", 
                            "ROBOT COMMAND", 
                            "GRIPPER STATE", 
                            "GRIPPER COMMAND", 
                            "MOTOR_STATE")