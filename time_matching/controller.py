import lcm
from robot_messages.frankalcm import robot_command, gripper_command
from motor_messages.motorlcm import motor_command
import time
import numpy as np

""" controller.py = sends the commands to the franka robot, gripper and the motor"""

class Controller:
    def __init__(self, rcm_channel, gcm_channel):

        # gripper states
        self.width = 0
        self.gripper_enabled = False

        # motor states
        self.motor_xmf = 0
        self.motor_ymf = 0
        self.motor_vel = 0
        self.motor_cur = 0

        # define lcm channels
        self.rcm_channel = rcm_channel
        self.gcm_channel = gcm_channel

        self.lc = lcm.LCM()

        # actions
        self.control_loop()
    
    def message(self, string):
        print("-----")
        print("controller.py: " + string)
        print("-----")

    def control_loop(self):
        start = time.time()
        self.message("loop started")

        loop_closed = False
        gripper_moved = False
        motion_finished = False
        motor_moved = False

        satellite_time = 7.6541
        motor_time = 1.
        robot_time = 1. + satellite_time - 1.#1.35439
        gripper_time = 1. + satellite_time - 0.5# - 0.6523

        while not loop_closed:
            rcm = robot_command()
            gcm = gripper_command()
            mcm = motor_command()

            # control logic --------------------------------------------------
            t = time.time() - start
            
            if (time.time()-start) >= gripper_time and gripper_moved == False:
                gcm.gripper_enable = True
                self.lc.publish(self.gcm_channel, gcm.encode())
                gripper_moved = True
            
            if (time.time()-start) >= robot_time and motion_finished == False:
                rcm.loop_open = True
                t_robot = t - robot_time
                q_d1 = -0.5 + 0.5*np.cos(np.pi * t_robot)
                rcm.q_d = np.array([q_d1, 0., 0., 0., 0., 0., 0.])
                rcm.motion_finished = False
                self.lc.publish(self.rcm_channel, rcm.encode())
                
                if t_robot >= 2.0:
                    rcm.motion_finished = True
                    print("motion finished")
                    self.lc.publish(self.rcm_channel, rcm.encode())
                    motion_finished = True
            
            # if (time.time()-start) >= motor_time and motor_moved == False:
            #     mcm.motor_enable = True
            #     self.lc.publish(self.mcm_channel, mcm.encode())
            #     motor_moved = True

            # if (time.time()-start) > 1.0 and gripper_moved == False:
            #     gripper_moved = True
            #     self.move_gripper(0.02, 10.0, 60.0)
            
            if (time.time()-start) > 10.0:
                rcm.loop_closed = True
                self.lc.publish(self.rcm_channel, rcm.encode())
                self.message("loop closed")
                loop_closed = True
    
    def move_gripper(self, width, speed, force):
        gcm = gripper_command()
        gcm.width = width
        gcm.speed = speed
        gcm.force = force
        self.lc.publish(self.gcm_channel, gcm.encode())
        self.message("gripper command sent")
        
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", "GRIPPER COMMAND")
