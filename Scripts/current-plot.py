import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
file_path = 'current_measure.csv'
data = pd.read_csv(file_path)

current = data['ch1']

print(np.nanmean(current))

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(current, label='Corrente', marker='o', color='goldenrod')
plt.ylim(0.10, 0.12)
plt.axhline(y=np.nanmean(current), color='cadetblue', linestyle='--', label='Média')
plt.axhline(y=np.nanmax(current), color='tomato', linestyle='--', label='Máximo')
# Add labels and title
plt.xlabel('Tempo (s)')
plt.ylabel('Corrente (A)')
plt.title('Medição de corrente')
plt.legend()
# Save the plot to a file
plt.savefig('graphs/current.png')