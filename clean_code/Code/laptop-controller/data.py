import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("cubli_log.csv")
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
for ax, col in zip(axs, ["theta_b", "theta_b_dot", "theta_w_dot"]):
    ax.plot(df["time_s"], df[col])
    ax.set_ylabel(col)

# Overlay raw (pre-clamp) vs. commanded (post-clamp) torque so saturation
# against kMaxTorqueNm shows up directly: if raw_torque routinely swings
# past the flat torque line, the LQR gains are demanding more than the
# clamp/motor can deliver.
axs[-1].plot(df["time_s"], df["raw_torque"], label="raw_torque", alpha=0.6)
axs[-1].plot(df["time_s"], df["torque"], label="torque")
axs[-1].set_ylabel("torque")
axs[-1].legend()

axs[-1].set_xlabel("time (s)")
plt.tight_layout()
plt.show()
