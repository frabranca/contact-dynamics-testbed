import lcm
from exlcm import robot_state, robot_command, gripper_command
import time
import numpy as np

class Controller:
    def __init__(self, robot_state_channel, robot_command_channel,  gripper_command_channel, save_output=False):
        # initiate state variables as zeros
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.loop_closed = False

        # lists to save output
        self.save_output = save_output
        self.tau_J_save = []
        self.time_save = []

        # define lcm channels
        self.robot_state_channel = robot_state_channel
        self.robot_command_channel = robot_command_channel
        self.gripper_command_channel = gripper_command_channel

        self.lc = lcm.LCM()
        self.subscription = self.lc.subscribe(self.robot_state_channel, self.robot_handler)

        # actions
        self.control_loop()
        #self.move_gripper()
        self.lc.unsubscribe(self.subscription)

        if save_output:
            self.write_output()

    def robot_handler(self, channel, data):
        rst = robot_state.decode(data)
        self.q           = rst.q
        self.q_d         = rst.q_d
        self.dq          = rst.dq
        self.dq_d        = rst.dq_d
        self.ddq_d       = rst.ddq_d
        self.tau_J       = rst.tau_J
        self.tau_J_d     = rst.tau_J_d
        self.dtau_J      = rst.dtau_J
        self.loop_closed = rst.loop_closed        

    def control_loop(self):
        start_time = time.time()
        while True:
            self.lc.handle()
            rcm = robot_command()

            # control logic
            rcm.tau_J_d = self.tau_J_d
            self.lc.publish(self.robot_command_channel, rcm.encode())

            if self.loop_closed:
                print("loop closed")
                break

            if round((time.time()-start_time), 3) == 10.0:
                self.move_gripper()
            
            if self.save_output:
                self.tau_J_save.append(self.tau_J)
                self.time_save.append(time.time() - start_time)
            
            # if (time.time() - start_time)>10.:
            #     break
    
    def move_gripper(self):
        #self.lc.handle()
        gcm = gripper_command()
        gcm.width = 0.02 
        gcm.speed = 10.0 
        gcm.force = 60.0
        self.lc.publish(self.gripper_command_channel, gcm.encode())
        print("gripper command sent")
    
    
    def write_output(self):
        output = open("output", "w")
        output.truncate()
        for i in range(len(self.tau_J_save)):
            output.write(str(self.time_save[i]) + ' ' + ' '.join(map(str, self.tau_J_save[i])) + '\n')
        output.close()
        
if __name__ == "__main__":
    controller = Controller("ROBOT STATE", "ROBOT COMMAND", "GRIPPER COMMAND")    
