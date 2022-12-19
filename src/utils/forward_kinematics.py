from math import *
import numpy as np
import pandas as pd

class ForwardKinematics:
    def __init__(self, theta_list):
        self.df = self.dataframe(theta_list)
        self.tm = np.round(self.tm(),4)

    def dataframe(self, theta_list):
        theta_list.append(0)
        dict = {"a": [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0],
                "d": [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107],
                "alpha": [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0],
                "theta": theta_list}

        idx = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", 
            "Joint 5", "Joint 6", "Joint 7", "Flange"]

        df = pd.DataFrame(dict, index = idx)
        return df
    
    def T(self, theta, alpha, a, d):
        rot = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha)],
                        [0, np.sin(alpha), np.cos(alpha)]])
        tran = np.array([[a*np.cos(theta),a*np.sin(theta),d]]).T

        zero_rot = np.eye(3)
        zero_tran = np.zeros((3,1))
        
        last_row = np.array([[0, 0, 0, 1]])

        Rotation    = np.vstack((np.hstack((rot, zero_tran)), last_row))
        Translation = np.vstack((np.hstack((zero_rot, tran)), last_row))
        return Rotation @ Translation
    
    def tm(self):
        tmi = np.eye(4)
        for i in self.df.index:
            tmi = tmi @ self.T(self.df["theta"][i], self.df["alpha"][i], self.df["a"][i], self.df["d"][i])
        return tmi

init_pose = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]

fk = ForwardKinematics(init_pose)
print(fk.tm)