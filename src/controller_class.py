import lcm
from exlcm import state, command
import time
import numpy as np

class Controller:
    def __init__(self):
        # initiate state variables as zeros
        self.q = 0
        self.q_d = 0
        self.dq = 0
        self.dq_d = 0
        self.ddq_d = 0
        self.tau_J = 0
        self.tau_J_d = 0
        self.dtau_J = 0
        self.width = 0.0
        self.max_width = 0.0
        self.is_grasped = False

        # define lcm channels
        self.channel_state = "STATE"
        self.channel_command = "COMMAND"
        self.lc = lcm.LCM()

        self.subscription = self.lc.subscribe(self.channel_state, self.my_handler)
        self.control_loop()
        self.lc.unsubscribe(self.subscription)

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
        try:
            while True:
                self.lc.handle()
                cmd = command()

                # control logic
                cmd.tau_J_d = self.tau_J_d
                self.lc.publish(self.channel_command, cmd.encode())

        except KeyboardInterrupt:
            pass 
        
if __name__ == "__main__":
    controller = Controller()    
