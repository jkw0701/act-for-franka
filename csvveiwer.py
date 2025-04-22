import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/home/namyoon/ACTfranka/joint_positions_only.csv")

plt.figure(figsize=(12, 6))
for j in range(7):
    plt.plot(df['time'].values, df[f'pos_j{j}'].values, label=f'Joint {j}')

plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.title("Joint Positions Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()