import numpy as np
import matplotlib.pyplot as plt

coefVel = 0.7
phiFinal_abs = 0.0

# Generate dmin values from 0 to 2
dmin_values = np.linspace(0, 2, 100)

# Calculate linearVel for each dmin value
linearVel = (coefVel * ((0.4 * np.log((3.5 * (dmin_values - 0.15)) + 0.0)) / (np.exp(0.883 * phiFinal_abs)) + (np.exp(1.57 - phiFinal_abs) / 6.5))) + 0.01

# Apply exponential reduction for dmin < 2
linearVel[dmin_values < 2.0] *= np.exp(-(2.5 - dmin_values[dmin_values < 2.0]))

# Plot linearVel vs dmin
plt.plot(dmin_values, linearVel)
plt.xlabel('dmin')
plt.ylabel('linearVel')
plt.title('linearVel vs dmin')
plt.grid(True)
plt.show()
