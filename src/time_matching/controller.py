import lcm
from robot_messages.frankalcm import robot_state, robot_command, gripper_command
from robot_messages.motorlcm import motor_command
import time
import matplotlib.pyplot as plt
import numpy as np

class Controller:
    def __init__(self, rst_channel, rcm_channel, gst_channel, gcm_channel, mst_channel, save_output=False, plot_data=False):
        # initiate state variables as zeros

        # robot states
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.robot_enabled = False

        # gripper states
        self.width = 0
        self.gripper_enabled = False

        # motor states
        self.motor_xmf = 0
        self.motor_ymf = 0
        self.motor_vel = 0
        self.motor_cur = 0

        # useful booleans
        self.gripper_moved = False
        self.plot_data = plot_data

        # lists to save output
        self.save_output = save_output
        self.tau_J_save = []
        self.time_save = []
        self.q_save = []
        self.qd_save = []

        # define lcm channels
        self.rst_channel = rst_channel
        self.rcm_channel = rcm_channel
        self.gcm_channel = gcm_channel
        self.gst_channel = gst_channel
        self.mst_channel = mst_channel

        # subscribe to channels
        self.lc = lcm.LCM()
        self.robot_sub   = self.lc.subscribe(self.rst_channel, self.robot_handler)
        # self.gripper_sub = self.lc.subscribe(self.gst_channel, self.gripper_handler)
        # self.motor_sub   = self.lc.subscribe(self.mst_channel, self.motor_handler)

        # actions

        self.control_loop()
        
        self.lc.unsubscribe(self.robot_sub)
        # self.lc.unsubscribe(self.gripper_sub)
        # self.lc.unsubscribe(self.motor_sub)
        
        if self.plot_data:
            self.tau_J_save = np.array(self.tau_J_save)
            self.plot()

        if save_output:
            self.write_output()

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
    
    def message(self, string):
        print("controller.py: " + string)

    def control_loop(self):
        start = time.time()
        loop_closed = False
        self.message("loop started")

        gcm_sent = False
        motion_finished = False
        mcm_sent = False

        satellite_time = 7.6541
        motor_time = 1.
        robot_time = 1. + satellite_time - 1.#1.35439
        gripper_time = 1. + satellite_time - 0.5# - 0.6523

        while not loop_closed:
            rcm = robot_command()
            gcm = gripper_command()
            mcm = motor_command()

            # control logic --------------------------------------------------
            #rcm.tau_J_d = self.tau_J_d
            t = time.time() - start

            #-----------------------------------------------------------------
            
            if (time.time()-start) >= gripper_time and gcm_sent == False:
                gcm.gripper_enable = True
                self.lc.publish(self.gcm_channel, gcm.encode())

                gcm_sent = True
            
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
                #rcm_sent = True
            
            # if (time.time()-start) >= motor_time and mcm_sent == False:
            #     mcm.motor_enable = True
            #     self.lc.publish(self.mcm_channel, mcm.encode())

            #     mcm_sent = True
            #self.lc.handle()
            if self.save_output:
                self.tau_J_save.append(self.tau_J)
                self.time_save.append(time.time() - start)
                self.q_save.append(self.q)
                self.qd_save.append(self.q_d)

            # if (time.time()-start) > 1.0 and self.gripper_moved == False:
            #     self.gripper_moved = True
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
    
    def plot(self):
        plt.figure()
        plt.plot(self.time_save, self.q_save)
        print(self.q_save)

        plt.legend()
        plt.grid()
        plt.show()
    
    def write_output(self):
        output = open("output", "w")
        output.truncate()
        for i in range(len(self.tau_J_save)):
            output.write(str(self.time_save[i]) + ' ' + ' '.join(map(str, self.tau_J_save[i])) + 
            ' ' + str(self.xyz_save[i][0]) + ' ' + str(self.xyz_save[i][1]) + ' ' + str(self.xyz_save[i][2]) + '\n')
        output.close()
        
if __name__ == "__main__":
    controller = Controller("ROBOT STATE", 
                            "ROBOT COMMAND", 
                            "GRIPPER STATE", 
                            "GRIPPER COMMAND", 
                            "MOTOR_STATE")
