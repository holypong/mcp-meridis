"""IMU comparison: real robot without vs with gyro feedback"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

REAL_PATH  = "log/buf_input-real.csv"
GYRO_PATH  = "log/buf_input-real-jyrofeedback.csv"

COL = {
    "AccX": 2, "AccY": 3, "AccZ": 4,
    "GyroX": 5, "GyroY": 6, "GyroZ": 7,
    "Roll": 12, "Pitch": 13, "Yaw": 14,
}

def load(path):
    return pd.read_csv(path, header=None)

def extract(df, col):
    return df.iloc[:, col].astype(float).values

def make_time(n, hz=100):
    return np.arange(n) / hz

def stats(arr):
    return f"mean={arr.mean():.3f}  std={arr.std():.3f}  p-p={np.ptp(arr):.3f}"

real = load(REAL_PATH)
gyro = load(GYRO_PATH)

groups = [
    ("Acceleration", ["AccX", "AccY", "AccZ"], ["m/s²", "m/s²", "m/s²"]),
    ("Gyro",         ["GyroX", "GyroY", "GyroZ"], ["rad/s", "rad/s", "rad/s"]),
    ("Attitude",     ["Roll", "Pitch", "Yaw"], ["deg", "deg", "deg"]),
]

fig = plt.figure(figsize=(18, 14))
fig.suptitle("IMU Comparison: Real (no feedback) vs Real (gyro feedback on)",
             fontsize=15, fontweight="bold", y=0.98)

gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.35)

plot_idx = 0
for group_name, keys, units in groups:
    for k, unit in zip(keys, units):
        ax = fig.add_subplot(gs[plot_idx // 3, plot_idx % 3])
        col = COL[k]

        r_arr = extract(real, col)
        g_arr = extract(gyro, col)
        t_r = make_time(len(r_arr))
        t_g = make_time(len(g_arr))

        ax.plot(t_r, r_arr, label="No feedback", color="#F44336", linewidth=1.2, alpha=0.85)
        ax.plot(t_g, g_arr, label="Gyro feedback", color="#4CAF50", linewidth=1.2, alpha=0.85)

        ax.set_title(f"{group_name} — {k} [{unit}]", fontsize=10)
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel(unit, fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

        r_stat = f"NO-FB {stats(r_arr)}"
        g_stat = f"GYRO  {stats(g_arr)}"
        ax.annotate(f"{r_stat}\n{g_stat}",
                    xy=(0.01, 0.02), xycoords="axes fraction",
                    fontsize=6.5, family="monospace",
                    verticalalignment="bottom",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7))

        plot_idx += 1

os.makedirs("report", exist_ok=True)
out = "report/imu_compare_real_vs_gyrofeedback.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved: {out}")
plt.show()
