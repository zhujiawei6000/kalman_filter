import matplotlib.pyplot as plt
import csv
import numpy as np

data = []
with open("data.csv") as f:
    rd = csv.reader(f, delimiter=",")
    for row in rd:
        data.append([float(s) for s in row])

# print(data)
data = np.array(data)
measure_theta = data[:, 0]
estimation_theta = data[:, 1]
estimation_theta_rate = data[:, 2]
print(measure_theta)
print(estimation_theta)
print(estimation_theta_rate)
plt.plot(estimation_theta, "bo-", label="Estimation")
plt.plot(measure_theta, "ro-", label="Measured")
plt.xlabel("Time (seconds)")
plt.ylabel("Theta (radians)")
plt.legend()
plt.show()
