import pandas as pd

import matplotlib.pyplot as plt

# Load the CSV file
file_path = 'model_bp5_predictions_embedded.csv'
data = pd.read_csv(file_path)

# Extract the required columns
# expected = data['Expected']
model_bp5 = data['model_bp5']
embedded = data['embedded']

# Plot the results
plt.figure(figsize=(10, 6))
# plt.plot(expected, label='Expected', marker='o')
plt.plot(embedded, label='Model on MCU', marker='s')
plt.plot(model_bp5, label='Model on desktop', marker='x')

# Add labels and title
plt.xlabel('Index')
plt.ylabel('Values')
plt.title('Comparison of inferences executed on PC and MCU')
plt.legend()
# Save the plot to a file
plt.savefig('comparison_plot.png')

# Show the plot
plt.show()