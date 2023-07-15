## 
# @file IR Sensor Fitting
# @brief This file contains the functions to fit the IR distance sensor data to a curve for use in the robocup project
# @author Jack Duignan (jdu80@uclive.ac.nz)

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# sensor data
sensor_distances = np.array([200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500])
sensor_rawData = np.array([580, 445, 340, 280, 245, 230, 214, 211, 210, 206])

