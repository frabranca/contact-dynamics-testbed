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
        self.t_save          = []
        self.q_save          = []
        self.dq_save         = []
        self.tau_save        = []
        self.ext_force_save = []
        self.EFpose_save     = []

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
        self.ext_force = 0
        self.EFpose = 0

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
        
        # controller gains  
        Kd_wait = np.array([1., 1., 1., 1., 1., 1., 1.])

        Kp_traj = np.array([2.7, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        Kd_traj = np.array([0.3, 0., 0., 0., 0., 0., 0.])

        Kd_damp = np.array([0.1, 0.1, 1.0, 0.1, 0.1, 0.1, 0.1])

        while not self.loop_closed:
            self.lc.handle()
            rcm = robot_command()
            gcm = gripper_command()
            mcm = motor_command()

            # control logic --------------------------------------------------
            t = time.time() - start
            t_robot = t - robot_time

            # WAITING PHASE
            if (t <= robot_time):
                rcm.robot_moving = False
                q_des  = np.array([0., 0., 0., 0., 0., 0., 0.])
                dq_des = np.array([0., 0., 0., 0., 0., 0., 0.])

                q_error  = q_des - self.q
                dq_error = dq_des - self.dq

                # robot acts as a damper to detumble satellite
                rcm.tau =  Kd_wait * dq_error
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            # TRAJECTORY PHASE
            if (t >= robot_time) and (t < robot_time + 2.0):
                q1_des  = 0.5 - 0.5*t_robot + 0.5 / np.pi *np.sin(np.pi * t_robot) - 0.05
                dq1_des = -0.5 + 0.5*np.cos(np.pi * t_robot)

                q_des = np.array([q1_des, 0., 0., 0., 0., 0., 0.])
                dq_des = np.array([dq1_des, 0., 0., 0., 0., 0., 0.])

                q_error = q_des - self.q
                dq_error = dq_des - self.dq

                rcm.tau = Kp_traj * q_error + Kd_traj * dq_error

                rcm.robot_moving = True
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            # GRIPPER COMMAND
            if (time.time()-start) >= gripper_time and gripper_moved == False:
                gripper_moved = True
                self.move_gripper(0.02, 10.0, 60.0)
            
            # MOTOR COMMAND
            if (time.time()-start) >= motor_time and motor_moved == False:
                mcm.motor_enable = True
                self.lc.publish(self.mcm_channel, mcm.encode())
                motor_moved = True
            
            # FINISH TEST
            if t > 20.0:
                self.loop_closed = True
            
            # DAMPING PHASE
            else:
                rcm.robot_moving = False
                dq_des = np.array([0., 0., 0., 0., 0., 0., 0.])

                dq_error = dq_des - self.dq

                # robot acts as a damper to detumble satellite
                rcm.tau = Kd_damp * dq_error
                self.lc.publish(self.rcm_channel, rcm.encode())

            if self.plot_data:
                self.t_save.append(t)
                self.q_save.append(self.q)
                self.dq_save.append(self.dq)
                self.tau_save.append(self.tau_J)
                self.ext_force_save.append(self.ext_force)
                self.EFpose_save.append(self.EFpose)
            
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
        self.robot_enable  = rst.robot_enable
        self.ext_force    = rst.ext_force
        self.EFpose        = rst.EFpose
    
    def move_gripper(self, width, speed, force):
        gcm = gripper_command()
        gcm.width = width
        gcm.speed = speed
        gcm.force = force
        self.lc.publish(self.gcm_channel, gcm.encode())
        self.message("gripper command sent")
    
    def write_data(self):
        output = open("joint_torques", "w")
        output.truncate()
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.tau_save[i])) + '\n')
        output.close()

        output = open("joint_velocities", "w")
        output.truncate()
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.dq_save[i])) + '\n')
        output.close()

        output = open("joint_positions", "w")
        output.truncate()
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.q_save[i])) + '\n')
        output.close()

        output = open("ext_force", "w")
        output.truncate()
        for i in range(len(self.ext_force_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.ext_force_save[i])) + '\n')
        output.close()

        output = open("EFpose", "w")
        output.truncate()
        for i in range(len(self.EFpose_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.EFpose_save[i])) + '\n')
        output.close()
    
    def show_plot(self):
        labels = ["1", "2", "3", "4", "5", "6", "7"]
        labels_ef = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]

        plt.figure()
        plt.plot(self.t_save, self.q_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint position [rad]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_positions.png")

        plt.figure()
        plt.plot(self.t_save, self.dq_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint velocity [rad/s]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_velocities.png")

            
        plt.figure()
        plt.plot(self.t_save, self.tau_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint torque [Nm]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_torques.png")


        plt.figure()
        plt.plot(self.t_save, self.ext_force_save)
        plt.xlabel("time [s]")
        plt.ylabel("external force on EF")
        plt.legend(labels_ef, loc="best")
        plt.grid()
        plt.savefig("end_effector_forces.png")
        
        plt.figure()
        plt.plot(self.t_save, self.EFpose_save)
        plt.xlabel("time [s]")
        plt.ylabel("EF position [m]")
        plt.legend(labels_ef, loc="best")
        plt.savefig("end_effector_position.png")
        plt.grid()

        plt.show()
        
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", 
                            "ROBOT STATE",
                            "GRIPPER COMMAND",
                            "MOTOR COMMAND", plot_data=True, save_data=True)
    
    if controller.plot_data==True:
        controller.t_save = np.array(controller.t_save)
        controller.q_save = np.array(controller.q_save)
        controller.dq_save = np.array(controller.dq_save)
        controller.tau_save = np.array(controller.tau_save) - np.mean(controller.tau_save[0:50], axis=0)
        controller.ext_force_save = np.array(controller.ext_force_save)
        controller.EFpose_save = np.array(controller.EFpose_save)
        controller.show_plot()