"""IMU comparison: single gyro gain vs independent roll/pitch gain"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

SINGLE_PATH = "log/buf_input-real-jyrofeedback.csv"   # mix_gyro_g=0.0002 (single)
SPLIT_PATH  = "log/buf_input-real-gyro-independent.csv"  # roll=0.0001, pitch=0.0002

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

single = load(SINGLE_PATH)
split  = load(SPLIT_PATH)

groups = [
    ("Acceleration", ["AccX", "AccY", "AccZ"], ["m/s²", "m/s²", "m/s²"]),
    ("Gyro",         ["GyroX", "GyroY", "GyroZ"], ["rad/s", "rad/s", "rad/s"]),
    ("Attitude",     ["Roll", "Pitch", "Yaw"], ["deg", "deg", "deg"]),
]

fig = plt.figure(figsize=(18, 14))
fig.suptitle(
    "IMU Comparison: Single gain (g=0.0002) vs Independent gain (roll=0.0001, pitch=0.0002)",
    fontsize=13, fontweight="bold", y=0.98
)

gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.35)

plot_idx = 0
for group_name, keys, units in groups:
    for k, unit in zip(keys, units):
        ax = fig.add_subplot(gs[plot_idx // 3, plot_idx % 3])
        col = COL[k]

        s_arr = extract(single, col)
        p_arr = extract(split, col)
        t_s = make_time(len(s_arr))
        t_p = make_time(len(p_arr))

        ax.plot(t_s, s_arr, label="Single g=0.0002", color="#F44336", linewidth=1.2, alpha=0.85)
        ax.plot(t_p, p_arr, label="Split roll/pitch", color="#4CAF50", linewidth=1.2, alpha=0.85)

        ax.set_title(f"{group_name} — {k} [{unit}]", fontsize=10)
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel(unit, fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

        s_stat = f"SINGLE {stats(s_arr)}"
        p_stat = f"SPLIT  {stats(p_arr)}"
        ax.annotate(f"{s_stat}\n{p_stat}",
                    xy=(0.01, 0.02), xycoords="axes fraction",
                    fontsize=6.5, family="monospace",
                    verticalalignment="bottom",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7))

        plot_idx += 1

os.makedirs("report", exist_ok=True)
out = "report/imu_compare_gyro_single_vs_split.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved: {out}")
plt.show()
