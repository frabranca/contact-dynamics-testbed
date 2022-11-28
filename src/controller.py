import lcm
from robot_messages.frankalcm import robot_state, robot_command, gripper_command, gripper_state
import time

class Controller:
    def __init__(self, rst_channel, rcm_channel, gst_channel, gcm_channel, save_output=False):
        # initiate state variables as zeros
        self.q   = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.width = 0
        self.robot_enabled = False
        self.gripper_enabled = False

        # useful booleans
        self.gripper_moved = False

        # lists to save output
        self.save_output = save_output
        self.tau_J_save = []
        self.time_save = []

        # define lcm channels
        self.rst_channel = rst_channel
        self.rcm_channel = rcm_channel
        self.gcm_channel = gcm_channel
        self.gst_channel = gst_channel

        # subscribe to channels
        self.lc = lcm.LCM()
        self.robot_sub = self.lc.subscribe(self.rst_channel, self.robot_handler)
        self.gripper_sub = self.lc.subscribe(self.gst_channel, self.gripper_handler)

        # actions
        self.lc.handle()
        if self.robot_enabled == True:
            self.control_loop()
        
        self.lc.unsubscribe(self.robot_sub)
        self.lc.unsubscribe(self.gripper_sub)

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
    
    def gripper_handler(self, channel, data):
        gst = gripper_state.decode(data)
        self.width           = gst.width 
        self.gripper_enabled = gst.gripper_enabled 
    
    def message(self, string):
        print("controller.py: " + string)

    def control_loop(self):
        start_time = time.time()
        loop_closed = False
        self.message("loop started")
        while not loop_closed:
            self.lc.handle()
            rcm = robot_command()
            # control logic
            rcm.tau_J_d = self.tau_J_d
            self.lc.publish(self.rcm_channel, rcm.encode())

            if self.save_output:
                self.tau_J_save.append(self.tau_J)
                self.time_save.append(time.time() - start_time)

            if (time.time()-start_time) > 10.0 and self.gripper_moved == False:
                self.gripper_moved = True
                self.move_gripper(0.02, 10.0, 60.0)
            
            if (time.time()-start_time) > 20.0:
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
    
    def write_output(self):
        output = open("src/output", "w")
        output.truncate()
        for i in range(len(self.tau_J_save)):
            output.write(str(self.time_save[i]) + ' ' + ' '.join(map(str, self.tau_J_save[i])) + '\n')
        output.close()
        
if __name__ == "__main__":
    controller = Controller("ROBOT STATE", "ROBOT COMMAND", "GRIPPER STATE", "GRIPPER COMMAND", save_output=True)    
