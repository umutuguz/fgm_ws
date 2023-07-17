import matplotlib.pyplot as plt
import numpy as np

coefVel = 1.0
phiFinal_abs = 0.0

# Generate a range of dmin_temp values
dmin_temp = np.linspace(0, 10, 100)

# Calculate linearVel using the equation
linearVel = (coefVel * ((0.4 * np.log((3.5 * (dmin_temp - 0.15)) + 0.0)) / (np.exp(0.883 * phiFinal_abs)) + (np.exp(1.57 - phiFinal_abs) / 6.5))) + 0.01

# Plot the data
plt.plot(dmin_temp, linearVel)
plt.xlabel('dmin_temp')
plt.ylabel('linearVel')
plt.title('linearVel vs. dmin_temp')
plt.grid(True)
plt.show()
