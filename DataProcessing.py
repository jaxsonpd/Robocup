import numpy as np
import matplotlib.pyplot as plt

# Read the data from SensorData.csv
data = np.loadtxt('SensorData.csv', delimiter=',', usecols=(0,1,2,3), skiprows=1)
topRaw = data[:, 0]
topFiltered = data[:, 1]
bottomRaw = data[:, 2]
bottomFiltered = data[:, 3]

# Plot the raw and filtered data for the whole data and then for a small snap shot on the same figure
plt.figure(1)
plt.subplot(211)
plt.plot(topRaw, label='Raw')
plt.plot(topFiltered, label='Filtered')
plt.ylim(0, 3000)
plt.xlim(0, None)

plt.ylabel('Distance (mm)')
plt.legend()

plt.subplot(212)
plt.plot(topRaw, label='Raw')
plt.plot(topFiltered, label='Filtered')
plt.ylim(1000, 2250)
plt.xlim(200, 300)
plt.xlabel('Time (Samples)')
plt.ylabel('Distance (mm)')
plt.legend()
plt.savefig('SensorFiltering.png')
plt.show()

