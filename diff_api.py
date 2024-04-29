import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file (replace 'file.csv' with your CSV file name)
data = pd.read_csv('./stats_file_kord_api.csv')  # Assuming there is no header

plt.figure(figsize=(10, 6))

loop_raw_time = data.iloc[:, 1] - data.iloc[:, 0]

plt.plot(loop_raw_time, label='waitSync', color='red', linestyle='-.')

plt.xlabel('Index')
plt.ylabel('Difference')
plt.title('API test')
plt.legend()
plt.show()
