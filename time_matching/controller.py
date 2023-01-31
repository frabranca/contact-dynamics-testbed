import lcm
from robot_messages.frankalcm import robot_command, robot_state, gripper_command
from motor_messages.motorlcm import motor_command
import time
import numpy as np
import matplotlib.pyplot as plt

""" controller.py = sends the commands to the franka robot, gripper and the motor"""

class Controller:
    def __init__(self, rcm_channel, rst_channel, gcm_channel, mcm_channel, plot_data=False, save_data=False):

        # booleans
        self.plot_data = plot_data
        self.save_data = save_data
        self.loop_closed = False

        # data lists
        self.t_save = []
        self.q_save = []
        self.dq_save = []
        self.tau_save = []

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

        # define lcm channels
        self.rcm_channel = rcm_channel
        self.rst_channel = rst_channel
        self.gcm_channel = gcm_channel
        self.mcm_channel = mcm_channel

        # subscribe to channels
        self.lc = lcm.LCM()
        self.robot_sub   = self.lc.subscribe(self.rst_channel, self.robot_handler)

        # actions
        self.lc.handle()
        if self.robot_enable == True:
            self.control_loop()
        
        self.lc.unsubscribe(self.robot_sub)

        if self.save_data:
            self.write_data()
        
        if self.plot_data==True:
            self.t_save = np.array(self.t_save)
            self.q_save = np.array(self.q_save)
            self.dq_save = np.array(self.dq_save)
            self.tau_save = np.array(self.tau_save)
            self.show_plot()
    
    def message(self, string):
        print("-----")
        print("controller.py: " + string)

    def control_loop(self):
        start = time.time()
        self.message("loop started")

        gripper_moved = False
        motor_moved = False

        satellite_time = 7.6541
        motor_time = 1.
        robot_time = 1. + satellite_time - 3.#1.35439
        gripper_time = 1. + satellite_time# - 0.6523

        while not self.loop_closed:
            Kp1 = np.array([2.7, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
            Kd1 = np.array([0.3, 0., 0., 0., 0., 0., 0.])
            Kd2 = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
            self.lc.handle()
            rcm = robot_command()
            gcm = gripper_command()
            mcm = motor_command()

            # control logic --------------------------------------------------
            t = time.time() - start
            t_robot = t - robot_time
            
            if (t >= robot_time) and (t < robot_time + 2.0):
                q1_des = 0.5 - 0.5*t_robot + 0.5 / np.pi *np.sin(np.pi * t_robot)
                dq1_des = -0.5 + 0.5*np.cos(np.pi * t_robot)

                q_des = np.array([q1_des, 0., 0., 0., 0., 0., 0.])
                dq_des = np.array([dq1_des, 0., 0., 0., 0., 0., 0.])

                q1_error = q_des - self.q
                dq1_error = dq_des - self.dq

                rcm.tau = Kp1 * q1_error + Kd1 * dq1_error

                rcm.robot_moving = True
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            if (time.time()-start) >= gripper_time and gripper_moved == False:
                gripper_moved = True
                self.move_gripper(0.02, 10.0, 60.0)
            
            if t > 20.0:
                self.loop_closed = True
            
            else:
                rcm.robot_moving = False
                dq_des = np.array([0., 0., 0., 0., 0., 0., 0.])

                dq1_error = dq_des - self.dq

                rcm.tau = Kd2 * dq1_error
                #rcm.tau = np.array([0., 0., 0., 0., 0., 0., 0.])
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            if (time.time()-start) >= motor_time and motor_moved == False:
                mcm.motor_enable = True
                self.lc.publish(self.mcm_channel, mcm.encode())
                motor_moved = True

            if self.plot_data:
                self.t_save.append(t)
                self.q_save.append(self.q)
                self.dq_save.append(self.dq)
                self.tau_save.append(self.tau_J)
            
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
        self.robot_enable = rst.robot_enable
    
    def move_gripper(self, width, speed, force):
        gcm = gripper_command()
        gcm.width = width
        gcm.speed = speed
        gcm.force = force
        self.lc.publish(self.gcm_channel, gcm.encode())
        self.message("gripper command sent")
    
    def write_data(self):
        output = open("output", "w")
        output.truncate()
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.tau_save[i])) + '\n')
        output.close()
    
    def show_plot(self):
        plt.figure()

        labels = ["1", "2", "3", "4", "5", "6", "7"]
        plt.subplot(131)
        plt.plot(self.t_save, self.q_save)
        plt.legend(labels)
        plt.grid()
        plt.subplot(132)
        plt.plot(self.t_save, self.dq_save)
        plt.grid()
        plt.subplot(133)
        plt.plot(self.t_save, self.tau_save)
        plt.grid()

        plt.show()
        
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", 
                            "ROBOT STATE",
                            "GRIPPER COMMAND",
                            "MOTOR COMMAND", plot_data=True)
