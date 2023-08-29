import numpy as np
import matplotlib.pyplot as plt

def calculate_influence(alpha_weight, dmin):
    weight = alpha_weight / np.exp(1.1*dmin*np.sqrt(dmin))
    return (weight / (weight + 1)) * 100

# Define the range of dmin values
dmin_values = np.linspace(0, 10, 100)  # Example range from 0 to 10

# Define the alpha_weight value
alpha_weight = 22
# Calculate corresponding influence percentages for each dmin value
influence_percentages = calculate_influence(alpha_weight, dmin_values)

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(dmin_values, influence_percentages, label='Influence of phi_gap')

# Adding labels and title
plt.xlabel('dmin')
plt.ylabel('Influence of phi_gap (%)')
plt.title('Influence of phi_gap vs. dmin')
plt.legend()

# Display the plot
plt.grid()
plt.show()
