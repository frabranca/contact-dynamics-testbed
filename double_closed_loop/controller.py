import lcm
from robot_messages.frankalcm import robot_command, robot_state, gripper_command
from motor_messages.motorlcm import motor_command, motor_state
import time
import numpy as np
import matplotlib.pyplot as plt

""" 
controller.py: main controller.

	- sends commands to 
        - torque_control.cpp (real-time "ROBOT COMMMAND")
        - gripper_control.cpp (non-real-time "GRIPPER COMMAND")
        - motor_controller.py (non-real-time "MOTOR COMMAND")
    - receives the state from torque_control.cpp (real-time "ROBOT STATE") 

"""

class Controller:
    def __init__(self, rcm_channel, rst_channel, mcm_channel, mst_channel, gcm_channel, plot_data=False, save_data=False):

        # booleans
        self.plot_data = plot_data
        self.save_data = save_data
        self.loop_closed = False

        # data lists
        self.t_save          = []
        self.q_save          = []
        self.q_d_save        = []
        self.dq_save         = []
        self.dq_d_save       = []
        self.tau_save        = []
        self.tau_d_save      = []
        self.ext_force_save  = []
        self.EFpose_save     = []

        self.sat_position_save = []
        self.sat_velocity_save = []

        # robot states
        self.q            = 0
        self.q_d          = 0
        self.dq           = 0
        self.dq_d         = 0
        self.ddq_d        = 0
        self.tau_J        = 0
        self.tau_J_d      = 0
        self.dtau_J       = 0
        self.robot_enable = 0
        self.ext_force    = 0
        self.EFpose       = 0

        # motor states
        self.sat_position = 0
        self.sat_velocity = 0

        # define lcm channels
        self.rcm_channel = rcm_channel
        self.rst_channel = rst_channel
        self.mcm_channel = mcm_channel
        self.mst_channel = mst_channel
        self.gcm_channel = gcm_channel

        # subscribe to channels
        self.lc = lcm.LCM()
        self.robot_subscription = self.lc.subscribe(self.rst_channel, self.robot_handler)
        self.motor_subscription = self.lc.subscribe(self.mst_channel, self.motor_handler)

        # actions
        self.lc.handle() # waiting for enabling message from torque_control.cpp
        if self.robot_enable == True:
            self.control_loop()
        
        self.lc.unsubscribe(self.robot_subscription)

        if self.save_data:
            self.write_data()
        
        if self.plot_data:
            self.tau_save   = np.array(self.tau_save) - np.mean(self.tau_save[0:50], axis=0)     # normalize measured torque
            self.tau_d_save = np.array(self.tau_d_save) - np.mean(self.tau_d_save[0:50], axis=0) # normalize desired torque
            self.show_plot()
    
    def message(self, string):
        print("-----")
        print("controller.py: " + string)

    def control_loop(self):
        start = time.time()

        robot_moved   = False
        gripper_moved = False
        motor_moved   = False

        # adjustable parameters
        satellite_time = 7.5
        motor_time     = 1.
        robot_time     = 1. + satellite_time - 3. # adjust robot starting time to reach contact point
        gripper_time   = 1. + satellite_time      # adjust gripper starting time to reach contact point
        
        q_fix = -0.05 # adjust position at the end of the trajectory to reach contact point
        gripper_width = 0.02
        gripper_speed = 10.0
        gripper_force = 60.0
        
        # controller gains  
        
        """ waiting phase """
        Kd_wait = np.array([1., 1., 1., 1., 1., 1., 1.])

        """ trajectory phase """
        Kp_traj = np.array([2.7, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        Kd_traj = np.array([0.3, 0., 0., 0., 0., 0., 0.])

        """ damping phase """
        Kd_damp = np.array([0.1, 0.1, 1.0, 0.1, 0.1, 0.1, 0.1])
        # Kd_damp = np.array([0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.5])

        self.message("loop started")
        t_ = 0
        while not self.loop_closed:
            self.lc.handle()
            rcm = robot_command()
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

                # robot acts as a damper hold the position before the trajectory is followed
                rcm.tau =  Kd_wait * dq_error
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            # TRAJECTORY PHASE
            # if (t >= robot_time) and (t < robot_time + 2.0):
            
            if self.sat_position <= -0.75 and  self.sat_position >= -1.0:
                t_ += 0.001
                q1_des  = 0.5 - 0.5*t_ + 0.5 / np.pi *np.sin(np.pi * t_) + q_fix
                dq1_des = -0.5 + 0.5*np.cos(np.pi * t_)

                q_des = np.array([q1_des, 0., 0., 0., 0., 0., 0.])
                dq_des = np.array([dq1_des, 0., 0., 0., 0., 0., 0.])

                q_error = q_des - self.q
                dq_error = dq_des - self.dq

                rcm.tau = Kp_traj * q_error + Kd_traj * dq_error

                if q1_des == -0.5 + q_fix:
                    robot_moved = True
                print(t_)
                rcm.robot_moving = True
                self.lc.publish(self.rcm_channel, rcm.encode())
            
            # GRIPPER COMMAND
            if (time.time()-start) >= gripper_time and gripper_moved == False:
                gripper_moved = True
                self.move_gripper(gripper_width, gripper_speed, gripper_force)
            
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
                # rcm.tau = np.array([0., 0., 0., 0., 0., 0., 0.])
                self.lc.publish(self.rcm_channel, rcm.encode())

            if self.plot_data:
                self.t_save.append(t)
                self.q_save.append(self.q)
                self.q_d_save.append(self.q_d)
                self.dq_save.append(self.dq)
                self.dq_d_save.append(self.dq_d)
                self.tau_save.append(self.tau_J)
                self.tau_d_save.append(self.tau_J_d)
                self.ext_force_save.append(self.ext_force)
                self.EFpose_save.append(self.EFpose)
                self.sat_position_save.append(self.sat_position)
                self.sat_velocity_save.append(self.sat_velocity)
            
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
        self.ext_force     = rst.ext_force
        self.EFpose        = rst.EFpose
    
    def motor_handler(self, channel, data):
        mst = motor_state.decode(data)
        self.sat_position = mst.position
        self.sat_velocity = mst.velocity
    
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
        columns = np.arange(1,8)
        columns = ["tau" + str(i) for i in columns]
        columns = ' '.join(map(str, columns))
        output.write("time" + ' ' + columns + '\n')
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.tau_save[i])) + '\n')
        output.close()

        output = open("joint_velocities", "w")
        output.truncate()
        columns = np.arange(1,8)
        columns = ["dq" + str(i) for i in columns]
        columns = ' '.join(map(str, columns))
        output.write("time" + ' ' + columns + '\n')
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.dq_save[i])) + '\n')
        output.close()

        output = open("joint_positions", "w")
        output.truncate()
        columns = np.arange(1,8)
        columns = ["q" + str(i) for i in columns]
        columns = ' '.join(map(str, columns))
        output.write("time" + ' ' + columns + '\n')
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.q_save[i])) + '\n')
        output.close()

        output = open("ext_force", "w")
        output.truncate()
        columns = "Fx Fy Fz Tx Ty Tz"
        output.write("time" + ' ' + columns + '\n')
        for i in range(len(self.ext_force_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.ext_force_save[i])) + '\n')
        output.close()

        output = open("EFpose", "w")
        output.truncate()
        columns = "x_ef y_ef z_ef"
        output.write("time" + ' ' + columns + '\n')
        for i in range(len(self.EFpose_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.EFpose_save[i])) + '\n')
        output.close()
    
    def show_plot(self):
        labels = ["1", "2", "3", "4", "5", "6", "7"]

        plt.figure()
        plt.plot(self.t_save, self.q_save)
        plt.plot(self.t_save, self.q_d_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint position [rad]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_positions.png")

        plt.figure()
        plt.plot(self.t_save, self.dq_save)
        plt.plot(self.t_save, self.dq_d_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint velocity [rad/s]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_velocities.png")
            
        plt.figure()
        plt.plot(self.t_save, self.tau_save)
        plt.plot(self.t_save, self.tau_d_save)
        plt.xlabel("time [s]")
        plt.ylabel("joint torque [Nm]")
        plt.legend(labels, loc="best")
        plt.grid()
        plt.savefig("joint_torques.png")

        plt.figure()
        plt.plot(self.t_save, self.ext_force_save)
        plt.xlabel("time [s]")
        plt.ylabel("external force on EF")
        plt.legend(["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"], loc="best")
        plt.grid()
        plt.savefig("end_effector_forces.png")
        
        plt.figure()
        plt.plot(self.t_save, self.EFpose_save)
        plt.xlabel("time [s]")
        plt.ylabel("EF position [m]")
        plt.legend(['x', 'y', 'z'], loc="best")
        plt.savefig("end_effector_position.png")
        plt.grid()

        plt.figure()
        plt.plot(self.t_save, self.sat_position_save)
        plt.xlabel("time [s]")
        plt.ylabel("satellite position [deg]")
        plt.savefig("satellite_position.png")
        plt.grid()

        # plt.show()
        
if __name__ == "__main__":
    controller = Controller("ROBOT COMMAND", 
                            "ROBOT STATE",
                            "MOTOR COMMAND", 
                            "MOTOR STATE",
                            "GRIPPER COMMAND",
                            plot_data=True, 
                            save_data=True)
