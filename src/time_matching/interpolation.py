import numpy as np
import matplotlib.pyplot as plt
q_init = np.array([0.0, 0.117397, -0.19942, -2.22072, -1.32267, 1.43232, 1.61111])
q_grasp = np.array([-0.48962, 0.117397, -0.19942, -2.22072, -1.32267, 1.43232, 1.61111])

q11 = 0.0
q12 = -0.48962
t = np.arange(0.,1., 0.001)
q1 = np.linspace(q11, q12, len(t))
time = 1.

def q(x):
    return 2.93772/3 * x**3 -2.93772/2 * x**2 

def qd(x):
    return 2.93772*x**2 - 2.93772*x

def qdd(x):
    return 2*2.93772*x - 2.93772

plt.plot(t, q(t))
plt.plot(t, qd(t))
plt.plot(t, qdd(t))

plt.show()