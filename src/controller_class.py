import lcm
from exlcm import state, command
import time
import numpy as np

class Controller:
    def __init__(self, channel_state_name, channel_command_name, save_output=False):
        # initiate state variables as zeros
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.width = 0
        self.max_width = 0
        self.is_grasped = False

        self.tau_J_save = []
        self.time_save = []
        self.save_output = save_output

        # define lcm channels
        self.channel_state = channel_state_name
        self.channel_command = channel_command_name
        self.lc = lcm.LCM()

        self.subscription = self.lc.subscribe(self.channel_state, self.my_handler)
        self.control_loop()
        self.lc.unsubscribe(self.subscription)

        if save_output:
            self.write_output()


    def my_handler(self, channel, data):
        st = state.decode(data)
        self.q          = st.q
        self.q_d        = st.q_d
        self.dq         = st.dq
        self.dq_d       = st.dq_d
        self.ddq_d      = st.ddq_d
        self.tau_J      = st.tau_J
        self.tau_J_d    = st.tau_J_d
        self.dtau_J     = st.dtau_J
        self.width      = st.width
        self.max_width  = st.max_width
        self.is_grasped = st.is_grasped

    def control_loop(self):
        start_time = time.time()
        while True:
            self.lc.handle()
            cmd = command()

            # control logic
            cmd.tau_J_d = self.tau_J_d
            self.lc.publish(self.channel_command, cmd.encode())
            
            if self.save_output:
                self.tau_J_save.append(self.tau_J)
                self.time_save.append(time.time() - start_time)
            if (time.time() - start_time)>10.:
                break

    
    def write_output(self):
        output = open("output", "w")
        for i in range(len(self.tau_J_save)):
            output.write(str(self.time_save[i]) + ' ' + ' '.join(map(str, self.tau_J_save[i])) + '\n')
        output.close()
        
if __name__ == "__main__":
    controller = Controller("STATE", "COMMAND", save_output=True)    
