"""
compare_gait_stability.py - 従来歩容 vs 新歩容の安定性比較

使い方:
    python tools/compare_gait_stability.py --base log/logs-XXXXXX.csv --new log/logs-YYYYYY.csv
    python tools/compare_gait_stability.py  # log/ 内の最新2ファイルを自動選択

出力:
    report/gait_stability_compare.png  グラフ比較
    コンソールに安定性指標サマリー
"""

import argparse
import os
import sys
import glob

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Meridim90 列インデックス（0-based）
COL = {
    "AccX": 2, "AccY": 3, "AccZ": 4,
    "GyroX": 5, "GyroY": 6, "GyroZ": 7,
    "Roll": 12, "Pitch": 13, "Yaw": 14,
}

REPORT_DIR = "report"


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, header=None)
    return df.astype(float)


def extract(df: pd.DataFrame, col: int) -> np.ndarray:
    return df.iloc[:, col].values


def make_time(n: int, hz: float = 100.0) -> np.ndarray:
    return np.arange(n) / hz


def stability_stats(arr: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(arr)),
        "std":  float(np.std(arr)),
        "p2p":  float(np.ptp(arr)),
        "max_abs": float(np.max(np.abs(arr))),
    }


def print_summary(label_base: str, label_new: str,
                  df_base: pd.DataFrame, df_new: pd.DataFrame):
    metrics = [
        ("Roll [deg]",  COL["Roll"]),
        ("Pitch [deg]", COL["Pitch"]),
        ("Yaw [deg]",   COL["Yaw"]),
        ("GyroX [rad/s]", COL["GyroX"]),
        ("GyroY [rad/s]", COL["GyroY"]),
        ("AccZ [m/s²]",  COL["AccZ"]),
    ]

    print("\n" + "=" * 72)
    print(f"  安定性比較サマリー")
    print(f"  Base : {label_base}  ({len(df_base)} frames)")
    print(f"  New  : {label_new}  ({len(df_new)} frames)")
    print("=" * 72)
    header = f"{'指標':<18} {'Base std':>10} {'New std':>10} {'改善':>8}  {'Base p-p':>10} {'New p-p':>10}"
    print(header)
    print("-" * 72)
    for name, col in metrics:
        b = stability_stats(extract(df_base, col))
        n = stability_stats(extract(df_new,  col))
        improve = (b["std"] - n["std"]) / (b["std"] + 1e-9) * 100
        sign = "↓" if improve > 0 else "↑"
        print(f"{name:<18} {b['std']:>10.4f} {n['std']:>10.4f} {sign}{abs(improve):>6.1f}%  "
              f"{b['p2p']:>10.4f} {n['p2p']:>10.4f}")
    print("=" * 72 + "\n")


def plot_compare(label_base: str, label_new: str,
                 df_base: pd.DataFrame, df_new: pd.DataFrame,
                 out_path: str):

    groups = [
        ("姿勢角 (Attitude)",
         [("Roll", "deg"), ("Pitch", "deg"), ("Yaw", "deg")]),
        ("ジャイロ (Gyro)",
         [("GyroX", "rad/s"), ("GyroY", "rad/s"), ("GyroZ", "rad/s")]),
        ("加速度 (Acceleration)",
         [("AccX", "m/s²"), ("AccY", "m/s²"), ("AccZ", "m/s²")]),
    ]

    t_base = make_time(len(df_base))
    t_new  = make_time(len(df_new))

    fig = plt.figure(figsize=(18, 13))
    fig.suptitle(
        f"Gait Stability Comparison\nBase: {os.path.basename(label_base)}  |  New: {os.path.basename(label_new)}",
        fontsize=13, fontweight="bold", y=0.99,
    )
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.35)

    for gi, (group_title, keys) in enumerate(groups):
        for ki, (key, unit) in enumerate(keys):
            ax = fig.add_subplot(gs[gi, ki])
            col = COL[key]
            b_arr = extract(df_base, col)
            n_arr = extract(df_new,  col)

            ax.plot(t_base, b_arr, color="steelblue",  alpha=0.8, lw=0.8, label=f"Base (σ={b_arr.std():.3f})")
            ax.plot(t_new,  n_arr, color="orangered",  alpha=0.8, lw=0.8, label=f"New  (σ={n_arr.std():.3f})")
            ax.set_title(f"{key} [{unit}]", fontsize=10)
            ax.set_xlabel("Time [s]", fontsize=8)
            ax.set_ylabel(unit, fontsize=8)
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)
            if ki == 0:
                ax.annotate(group_title, xy=(-0.25, 0.5), xycoords="axes fraction",
                            fontsize=9, fontweight="bold", rotation=90, va="center")

    # 安定性スコアバー（Roll std を代表指標として可視化）
    ax_bar = fig.add_axes([0.35, 0.01, 0.30, 0.04])
    roll_b = extract(df_base, COL["Roll"]).std()
    roll_n = extract(df_new,  COL["Roll"]).std()
    pitch_b = extract(df_base, COL["Pitch"]).std()
    pitch_n = extract(df_new,  COL["Pitch"]).std()
    labels = ["Roll std", "Pitch std"]
    x = np.arange(len(labels))
    ax_bar.bar(x - 0.2, [roll_b, pitch_b], 0.35, color="steelblue", label="Base")
    ax_bar.bar(x + 0.2, [roll_n, pitch_n], 0.35, color="orangered",  label="New")
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(labels, fontsize=7)
    ax_bar.set_ylabel("deg", fontsize=7)
    ax_bar.set_title("姿勢安定性 (低いほど安定)", fontsize=8)
    ax_bar.legend(fontsize=7)
    ax_bar.tick_params(labelsize=7)

    os.makedirs(REPORT_DIR, exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    print(f"グラフ保存: {out_path}")
    plt.show()


def pick_latest_two() -> tuple[str, str]:
    """log/ 内の logs-*.csv を更新日時順で2件取得（古い=base, 新しい=new）"""
    files = sorted(glob.glob("log/logs-*.csv"))
    if len(files) < 2:
        print("error: log/ に logs-*.csv が2件以上必要です。", file=sys.stderr)
        sys.exit(1)
    return files[-2], files[-1]


def main():
    parser = argparse.ArgumentParser(
        description="従来歩容 vs 新歩容の安定性比較",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--base", type=str, default=None,
                        help="従来歩容のログ CSV（省略時: log/ 内の古い方）")
    parser.add_argument("--new",  type=str, default=None,
                        help="新歩容のログ CSV（省略時: log/ 内の新しい方）")
    parser.add_argument("--out",  type=str, default="report/gait_stability_compare.png",
                        help="出力グラフパス")
    args = parser.parse_args()

    if args.base is None or args.new is None:
        base_path, new_path = pick_latest_two()
        print(f"自動選択: base={base_path}, new={new_path}")
    else:
        base_path, new_path = args.base, args.new

    for p in (base_path, new_path):
        if not os.path.exists(p):
            print(f"error: ファイルが見つかりません: {p}", file=sys.stderr)
            sys.exit(1)

    df_base = load_csv(base_path)
    df_new  = load_csv(new_path)

    print_summary(base_path, new_path, df_base, df_new)
    plot_compare(base_path, new_path, df_base, df_new, args.out)


if __name__ == "__main__":
    main()
