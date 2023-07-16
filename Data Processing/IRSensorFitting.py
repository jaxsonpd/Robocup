## 
# @file IR Sensor Fitting
# @brief This file contains the functions to fit the IR distance sensor data to a curve for use in the robocup project
# @author Jack Duignan (jdu80@uclive.ac.nz)

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# sensor data
sensor_distances = np.array([200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100])
sensor_rawData = np.array([580, 445, 340, 280, 245, 230, 214, 211, 210, 206])

# fit the data
a, b = np.polyfit(sensor_distances, sensor_rawData, 1)

# plot the fit
plt.plot(sensor_distances, sensor_rawData, 'bo', label='Raw Data')
plt.plot(sensor_distances, a*sensor_distances + b, 'r-', label='Fit')
plt.legend()
plt.show()