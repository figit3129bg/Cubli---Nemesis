import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("cubli_log.csv")
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
for ax, col in zip(axs, ["theta_b", "theta_b_dot", "theta_w_dot", "torque"]):
    ax.plot(df["time_s"], df[col])
    ax.set_ylabel(col)
axs[-1].set_xlabel("time (s)")
plt.tight_layout()
plt.show()

corr = np.correlate(df["theta_b"], df["torque"], mode='full')
lag = np.argmax(corr)-len(df["theta_b"])+1
delay = lag * df["time_s"].iloc[1] - df["time_s"].iloc[0]
print(f"Estimated delay: {delay:.3f} seconds")

theta_std = np.std(df["theta_b"])
gyro_std = np.std(df["theta_b_dot"])
wheel_std = np.std(df["theta_w_dot"])

print(f"Standard deviation of theta_b: {theta_std:.4f} rad")
print(f"Standard deviation of theta_b_dot: {gyro_std:.4f} rad/s")
print(f"Standard deviation of theta_w_dot: {wheel_std:.4f} rad/s")
