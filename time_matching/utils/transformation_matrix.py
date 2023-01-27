import numpy as np
from math import *

def T(w, th, p):
    p = np.array([p]).T
    cth = np.cos(th)
    sth = np.sin(th)
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    R11 = cth + w1**2*(1-cth)
    R12 = w1*w2*(1-cth)-w3*sth
    R13 = w1*w3*(1-cth) + w2*sth
    R21 = w1*w2*(1-cth) + w3*sth
    R22 = cth + w2**2*(1-cth)
    R23 = w2*w3*(1-cth) - w1*sth
    R31 = w1*w3*(1-cth) - w2*sth
    R32 = w2*w3*(1-cth) + w1*sth
    R33 = cth + w3**2*(1-cth)
    R = np.array([[R11, R12, R13],
                [R21, R22, R23],
                [R31, R32, R33]])
    row = np.array([0,0,0,1])
    T = np.vstack((np.hstack((R, p)),row))
    return np.round(T,3)

print(T((0,0,2), pi/2, (0,2,0)))
