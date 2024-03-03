import matplotlib.pyplot as plt
import csv
import numpy as np
data = []
with open('data.csv') as f:
    rd = csv.reader(f, delimiter=',')
    for row in rd:
        data.append([float(s) for s in row])

print(data)
data = np.array(data)
measure_altitude = data[:, 0]
estimation_altitude = data[:, 1]
estimation_velocity = data[:, 2]

plt.subplot(2,1,1)
plt.plot(measure_altitude, 'ro-', label='measurements')
plt.plot(estimation_altitude, 'bo-', label='estimates')
plt.xlabel('tick')
plt.ylabel('altitude')
plt.legend()
plt.subplot(2,1,2)
plt.plot(estimation_velocity, 'bo-', label='estimates')
plt.xlabel('tick')
plt.ylabel('velocity')

plt.legend()
plt.show()