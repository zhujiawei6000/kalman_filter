import matplotlib.pyplot as plt
import csv
import numpy as np

data = []
with open("data.csv") as f:
    rd = csv.reader(f, delimiter=",")
    for row in rd:
        data.append([float(s) for s in row])

print(data)
data = np.array(data)
measure_x = data[:, 0]
measure_y = data[:, 1]
estimation_x = data[:, 2]
estimation_y = data[:, 3]
estimation_vx = data[:, 4]
estimation_vy = data[:, 5]
print(measure_x)
print(measure_y)
print(estimation_x)
print(estimation_y)
plt.subplot(2, 2, (1, 3))
plt.plot(measure_x, measure_y, "ro-", label="measurements")
plt.plot(estimation_x, estimation_y, "bo-", label="estimates")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.subplot(2, 2, 2)
plt.plot(estimation_vx, "bo-", label="estimates")
plt.ylabel("vx")
plt.xlabel("time")
plt.legend()
plt.subplot(2, 2, 4)
plt.plot(estimation_vy, "bo-", label="estimates")
plt.ylabel("vy")
plt.xlabel("time")
plt.legend()
plt.show()
