import numpy as np

q = np.array([[1,2],[3,4],[5,6],[7,8],[9,10],[11,12],[13,14]])
print(q[0:3])
avg = np.mean(q[0:3], axis=0)
print(avg)
