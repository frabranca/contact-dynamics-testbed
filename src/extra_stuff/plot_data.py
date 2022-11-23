import matplotlib.pyplot as plt
import pandas as pd

output = open("output_moved")
df = pd.read_csv(output, header=0, delimiter=' ')

plt.plot(df["time"], df["tau1"], label="tau1")
plt.plot(df["time"], df["tau2"], label="tau2")
plt.plot(df["time"], df["tau3"], label="tau3")
plt.plot(df["time"], df["tau4"], label="tau4")
plt.plot(df["time"], df["tau5"], label="tau5")
plt.plot(df["time"], df["tau6"], label="tau6")
plt.plot(df["time"], df["tau7"], label="tau7")

plt.grid()
plt.legend(loc='best')

plt.show()