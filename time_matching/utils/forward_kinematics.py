from math import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class ForwardKinematics:
    def __init__(self, theta_list):
        self.df = self.dataframe(theta_list)
        self.tm = self.tmatrix()[0]
        self.xyz = self.tmatrix()[1]

    def dataframe(self, theta_list):
        theta_list.append(0)
        dict = {"a": [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0],
                "d": [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107],
                "alpha": [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0],
                "theta": theta_list}
        
        # dict = {"a": [    0,     0,     0,    0.0825, -0.0825,      0,  0.088,    0],
        #         "d": [0.135, 0.198, 0.195,     0.121,    0.11,  0.274,      0, .107],
        #         "alpha": [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0],
        #         "theta": theta_list}

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
    
    def tmatrix(self):
        tmi = np.eye(4)
        xyz = []
        for i in self.df.index:
            tmi = tmi @ self.T(self.df["theta"][i], self.df["alpha"][i], self.df["a"][i], self.df["d"][i])
            xyzi = np.round(tmi,4)[0:3,3:4].T[0]
            xyz.append(xyzi)
        xyz = np.array(xyz)
        tmi = np.round(tmi,4)
        return tmi, xyz

init_pose = [0, -pi/4, 0, -3*pi/4, -pi/2, pi/2, 3*pi/4]
# init_pose = [0,0,pi/4,0,0,0,0]

fk = ForwardKinematics(init_pose)
x = fk.xyz[:,0]
y = fk.xyz[:,1]
z = fk.xyz[:,2]

print(fk.tm)

ax = plt.axes(projection='3d')
ax.plot3D(x,y,z)
ax.set_xlim([0,1])
ax.set_ylim([0,1])
ax.set_zlim([0,1])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.show()

