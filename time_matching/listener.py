import lcm
from robot_messages.frankalcm import robot_state
import time
import matplotlib.pyplot as plt
import numpy as np

""" listener.py = only listens to the states of the franka robot through the ROBOT_STATE channel"""

class Listener:
    def __init__(self, rst_channel, listen_time, plot_data=False, save_output=False):
        self.rst_channel = rst_channel
        self.listen_time = listen_time
        self.q_save = []
        self.dq_save = []
        self.tau_save = []
        self.t_save = []

        self.lc = lcm.LCM()
        self.robot_sub   = self.lc.subscribe(rst_channel, self.robot_handler)

        self.listen()

        self.q_save = np.array(self.q_save)
        self.dq_save = np.array(self.dq_save)
        self.tau_save = np.array(self.tau_save)

        if plot_data == True:
            self.plot()

        if save_output == True:
            self.write_output()   
    
    def listen(self):
        start = time.time()
        while True:
            if (time.time()-start)<10.:
                self.t_save.append(time.time()-start)
                self.lc.handle()
            else:
                break

    def robot_handler(self, channel, data):
        rst = robot_state.decode(data)
        self.q_save.append(rst.q)
        self.dq_save.append(rst.dq)
        self.tau_save.append(rst.tau_J)
    
    def write_output(self):
        output = open("output", "w")
        output.truncate()
        for i in range(len(self.tau_save)):
            output.write(str(self.t_save[i]) + ' ' + ' '.join(map(str, self.tau_save[i])) + '\n')
        output.close()

    def plot(self):
        plt.figure()
        
        plt.subplot(131)
        plt.xlabel("time [s]")
        plt.ylabel("q [rad]")
        plt.plot(self.t_save, self.q_save[:,0])
        
        plt.subplot(132)
        plt.xlabel("time [s]")
        plt.ylabel("dq [rad/s]")
        plt.plot(self.t_save, self.dq_save[:,0])
        
        plt.subplot(133)
        plt.xlabel("time [s]")
        plt.ylabel("tau [Nm]")
        plt.plot(self.t_save, self.tau_save[:,0])

        plt.show()

if __name__ == "__main__":
    listener = Listener("ROBOT STATE", 10., plot_data=True, save_output=True)