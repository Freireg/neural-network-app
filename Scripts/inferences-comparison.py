import pandas as pd

import matplotlib.pyplot as plt

# Load the CSV file
file_path = 'model_bp5_predictions_embedded.csv'
data = pd.read_csv(file_path)

# Extract the required columns
expected = data['Expected']
model_bp5 = data['model_bp5']
embedded = data['embedded']


# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(expected, label='Real', marker='o', color='steelblue')
plt.plot(model_bp5, label='Keras', marker='^', color='darkorange')

# Add labels and title
plt.xlabel('Indices')
plt.ylabel('Valor')
plt.title('Valor Real x Valor Previsto Keras')
plt.legend()
# Save the plot to a file
plt.savefig('graphs/RealxKeras.png')

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(model_bp5, label='Keras', marker='^', color='darkorange')
plt.plot(embedded, label='MCU', marker='x', color='green')

# Add labels and title
plt.xlabel('Indices')
plt.ylabel('Valor')
plt.title('Inferência no PC x Inferência no MCU')
plt.legend()
# Save the plot to a file
plt.savefig('graphs/KerasxMCU.png')

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(expected, label='Real', marker='o', color='steelblue')
plt.plot(model_bp5, label='Keras', marker='^', color='darkorange')
plt.plot(embedded, label='MCU', marker='x', color='green')

# Add labels and title
plt.xlabel('Indices')
plt.ylabel('Valor')
plt.title('Valor Real x Inferência Keras x Inferência MCU')
plt.legend()
# Save the plot to a file
plt.savefig('graphs/RealxKerasxMCU.png')

# Show the plot
# plt.show()