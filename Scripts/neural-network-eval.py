import pandas as pd
import serial
import time

import matplotlib.pyplot as plt

# Load the CSV file
csv_file = 'model_bp5_predictions_desktop.csv'
data = pd.read_csv(csv_file)

# Extract the required columns
samples = data[['Ammonia', 'Nitrate', 'Temperature', 'pH']]
expected_values = data['Expected']
model_bp5_values = data['model_bp5']

# Initialize serial communication
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# time.sleep(2)  # Wait for the serial connection to initialize

inference_results = []

# Send samples over serial and collect inference results
for index, row in samples.iterrows():
  sample_str = f"{row['Ammonia']} {row['Nitrate']} {row['Temperature']} {row['pH']}"
  # print(sample_str)
  ser.write(sample_str.encode())
  
  # Wait for a response
  time.sleep(10.5)
  response = ser.readline().decode().strip()
  # print(response[:5])
  if response:
    inference_result = float(response[:5])
    inference_results.append(inference_result)
  else:
    inference_results.append(None)

# Close the serial connection
ser.close()

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(expected_values, label='Expected', marker='o')
plt.plot(model_bp5_values, label='Model BP5', marker='x')
plt.plot(inference_results, label='Inference Results', marker='s')
plt.xlabel('Sample Index')
plt.ylabel('Values')
plt.title('Comparison of Expected, Model BP5, and Inference Results')
plt.legend()
plt.savefig('inferences_comparison_plot.png')
plt.show()