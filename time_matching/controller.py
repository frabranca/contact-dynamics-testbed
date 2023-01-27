import lcm
from robot_messages.frankalcm import robot_command, robot_state, gripper_command
from motor_messages.motorlcm import motor_command
import time
import numpy as np

""" controller.py = sends the commands to the franka robot, gripper and the motor"""

class Controller:
    def __init__(self, rcm_channel, rst_channel, gcm_channel):

        # robot states
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.robot_enable = False

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
        self.rst_channel = rst_channel
        self.gcm_channel = gcm_channel

        # subscribe to channels
        self.lc = lcm.LCM()
        self.robot_sub   = self.lc.subscribe(self.rst_channel, self.robot_handler)

        # actions
        self.lc.handle()
        if self.robot_enabled == True:
            self.control_loop()
        
        self.lc.unsubscribe(self.robot_sub)

    
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
            self.lc.handle()
            rcm = robot_command()
            gcm = gripper_command()
            mcm = motor_command()

            # control logic --------------------------------------------------
            t = time.time() - start
            print(self.q[0])
            
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
        self.robot_enabled = rst.robot_enabled
        self.pose          = rst.pose
    
    def move_gripper(self, width, speed, force):
        gcm = gripper_command()
        gcm.width = width
        gcm.speed = speed
        gcm.force = force
        self.lc.publish(self.gcm_channel, gcm.encode())
        self.message("gripper command sent")
        
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", 
                            "ROBOT STATE",
                            "GRIPPER COMMAND")
