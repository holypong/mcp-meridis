"""IMU comparison: simulator vs real robot walk logs"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import sys
import os

SIM_PATH  = "log/buf_input-simulator.csv"
REAL_PATH = "log/buf_input-real.csv"

# Meridim90 column indices (0-based)
COL = {
    "AccX": 2, "AccY": 3, "AccZ": 4,
    "GyroX": 5, "GyroY": 6, "GyroZ": 7,
    "Roll": 12, "Pitch": 13, "Yaw": 14,
}

def load(path):
    df = pd.read_csv(path, header=None)
    return df

def extract(df, col):
    return df.iloc[:, col].astype(float).values

def make_time(n, hz=100):
    return np.arange(n) / hz

def stats(arr):
    return f"mean={arr.mean():.3f}  std={arr.std():.3f}  p-p={np.ptp(arr):.3f}"

sim  = load(SIM_PATH)
real = load(REAL_PATH)

groups = [
    ("加速度", ["AccX", "AccY", "AccZ"], ["m/s²", "m/s²", "m/s²"]),
    ("ジャイロ", ["GyroX", "GyroY", "GyroZ"], ["rad/s", "rad/s", "rad/s"]),
    ("姿勢角", ["Roll", "Pitch", "Yaw"], ["deg", "deg", "deg"]),
]

fig = plt.figure(figsize=(18, 14))
group_en = {"加速度": "Acceleration", "ジャイロ": "Gyro", "姿勢角": "Attitude"}
fig.suptitle("IMU Comparison: Simulator vs Real Robot (Walk 5s)", fontsize=15, fontweight="bold", y=0.98)

total_plots = 9
gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.35)

plot_idx = 0
for group_name, keys, units in groups:
    for k, unit in zip(keys, units):
        ax = fig.add_subplot(gs[plot_idx // 3, plot_idx % 3])
        col = COL[k]

        s_arr = extract(sim,  col)
        r_arr = extract(real, col)
        t_s   = make_time(len(s_arr))
        t_r   = make_time(len(r_arr))

        ax.plot(t_s, s_arr, label="Simulator", color="#2196F3", linewidth=1.2, alpha=0.85)
        ax.plot(t_r, r_arr, label="Real",      color="#F44336", linewidth=1.2, alpha=0.85)

        ax.set_title(f"{group_en[group_name]} — {k} [{unit}]", fontsize=10)
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel(unit, fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

        # annotation
        s_stat = f"SIM  {stats(s_arr)}"
        r_stat = f"REAL {stats(r_arr)}"
        ax.annotate(f"{s_stat}\n{r_stat}",
                    xy=(0.01, 0.02), xycoords="axes fraction",
                    fontsize=6.5, family="monospace",
                    verticalalignment="bottom",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7))

        plot_idx += 1

out = "report/imu_compare_sim_vs_real.png"
os.makedirs("report", exist_ok=True)
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved: {out}")
plt.show()
