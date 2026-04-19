#!/usr/bin/env python3
"""
estimate_walk_params.py
前進歩行ログ（redis_logger.py出力）から歩容パラメータを推定し、
walkparam.json と同形式で log/ ディレクトリに出力する。

使い方:
    python tools/estimate_walk_params.py --log log/logs-202604190935.csv
    python tools/estimate_walk_params.py  # 最新のlogsファイルを自動選択

推定対象パラメータ:
  forward_stride       : 足先X方向振幅 [m] (foot_x の振幅から)
  foot_lift            : 遊脚高さ [m]       (foot_z の最大値から)
  hip_swing            : 横スイング量 [m]   (hip_roll 角度から逆算)
  forward_lean_angle   : 前傾角度 [度]      (大腿・足首・膝の角度対称性から)
  cycle_duration       : 1周期の時間 [s]    (foot_x への正弦波フィット)
  swing_ratio          : 遊脚期間比率       (foot_z > 0 の期間比率)

その他のパラメータは walkparam.json の値を引き継ぐ。
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.optimize import curve_fit

# ── Meridim90 カラムインデックス（0 始まり） ──────────────────────────
COL_TIMESTAMP  = 1
COL_PAD_BTN    = 15
COL_FWD_INPUT  = 17   # 前進操作量 (>0.1 で前進歩行と判定)
COL_L_HIP_ROLL = 33
COL_L_THIGH    = 35
COL_L_KNEE     = 37
COL_L_ANKLE_P  = 39
COL_L_FOOT_X   = 47   # L 足先 X 位置 [mm]
COL_L_FOOT_Z   = 49   # L 足先 Z 位置（遊脚高さ）[mm]
COL_R_HIP_ROLL = 63
COL_R_THIGH    = 65
COL_R_KNEE     = 67
COL_R_ANKLE_P  = 69
COL_R_FOOT_X   = 77   # R 足先 X 位置 [mm]
COL_R_FOOT_Z   = 79   # R 足先 Z 位置（遊脚高さ）[mm]

MM2M = 1.0 / 1000.0   # mm → m 換算係数

# Meridim90 シーケンス番号の時間換算（100Hz = 10ms/count）
FRAME_RATE   = 100.0           # Hz
FRAME_PERIOD = 1.0 / FRAME_RATE  # 0.010 s/count

# リンク長デフォルト（LinkParams デフォルト値）
THIGH  = 0.065   # [m]
SHANK  = 0.065   # [m]
SHORTEN = 0.020  # [m]
LEG_Z  = THIGH + SHANK - SHORTEN   # 立位時の脚長 ≈ 0.110 [m]

LOG_DIR      = "log"
WALKPARAM    = "walkparam.json"
FWD_THRESH   = 0.1    # col[17] の前進判定しきい値


# ─────────────────────────────────────────────────────────────────────
# データ読み込み
# ─────────────────────────────────────────────────────────────────────
def load_csv(path: str) -> list:
    data = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                data.append([float(v) for v in line.split(",")])
            except ValueError:
                pass
    return data


def find_latest_log() -> str:
    files = sorted(glob.glob(os.path.join(LOG_DIR, "logs-*.csv")))
    if not files:
        print("Error: ログファイルが見つかりません。--log オプションで指定してください。",
              file=sys.stderr)
        sys.exit(1)
    return files[-1]


# ─────────────────────────────────────────────────────────────────────
# 前進歩行区間の抽出
# ─────────────────────────────────────────────────────────────────────
def filter_forward(rows: list) -> list:
    """col[17] > FWD_THRESH の行のみ返す"""
    return [r for r in rows if r[COL_FWD_INPUT] > FWD_THRESH]


def unique_by_timestamp(rows: list):
    """同一タイムスタンプの重複行を除去して配列を返す"""
    seen = set()
    uniq = []
    for r in rows:
        ts = r[COL_TIMESTAMP]
        if ts not in seen:
            seen.add(ts)
            uniq.append(r)
    return uniq


# ─────────────────────────────────────────────────────────────────────
# 各パラメータの推定
# ─────────────────────────────────────────────────────────────────────
def est_forward_stride(rows: list) -> float:
    """
    foot_x の peak-to-peak 振幅の 1/2 = forward_stride。
    l_foot_x = -forward_stride * sin(phase) なので振幅 = forward_stride。
    左右の平均を返す。
    """
    l_fx = np.array([r[COL_L_FOOT_X] * MM2M for r in rows])
    r_fx = np.array([r[COL_R_FOOT_X] * MM2M for r in rows])
    l_amp = (l_fx.max() - l_fx.min()) / 2.0
    r_amp = (r_fx.max() - r_fx.min()) / 2.0
    return float(np.mean([l_amp, r_amp]))


def est_foot_lift(rows: list) -> float:
    """
    foot_z の最大値 = foot_lift。
    l_foot_z = foot_lift * sin(swing_phase * pi) なので max = foot_lift。
    """
    l_fz = np.array([r[COL_L_FOOT_Z] * MM2M for r in rows])
    r_fz = np.array([r[COL_R_FOOT_Z] * MM2M for r in rows])
    return float(max(l_fz.max(), r_fz.max()))


def est_hip_swing(rows: list) -> float:
    """
    hip_roll 角度から横スイング量を逆算。
    IK: hip_roll = arctan2(target_y, sqrt(target_y² + target_z²))
    target_y = ±hip_swing（peak 時）, target_z ≈ LEG_Z
    → hip_swing ≈ tan(max_hip_roll) * LEG_Z
    左右の最大 |hip_roll| の平均を使用。
    """
    l_roll = np.array([r[COL_L_HIP_ROLL] for r in rows])
    r_roll = np.array([r[COL_R_HIP_ROLL] for r in rows])
    max_roll_deg = np.mean([np.abs(l_roll).max(), np.abs(r_roll).max()])
    return float(np.tan(np.radians(max_roll_deg)) * LEG_Z)


def est_forward_lean_angle(rows: list) -> float:
    """
    大腿ピッチ・膝ピッチ・足首ピッチの幾何学的対称性から前傾角度を推定。
    IK の関係式:
        thigh_IK + ankle_IK = -knee_pitch
    walk_ctrl では前傾角を大腿に加算:
        thigh_obs = thigh_IK - lean_angle
    よって:
        lean_angle = -(thigh_obs + ankle_obs + knee_obs)  の中央値
    """
    l_thigh  = np.array([r[COL_L_THIGH]   for r in rows])
    l_knee   = np.array([r[COL_L_KNEE]    for r in rows])
    l_ankle  = np.array([r[COL_L_ANKLE_P] for r in rows])
    r_thigh  = np.array([r[COL_R_THIGH]   for r in rows])
    r_knee   = np.array([r[COL_R_KNEE]    for r in rows])
    r_ankle  = np.array([r[COL_R_ANKLE_P] for r in rows])

    lean_l = -(l_thigh + l_ankle + l_knee)
    lean_r = -(r_thigh + r_ankle + r_knee)
    lean = float(np.median(np.concatenate([lean_l, lean_r])))
    return float(np.clip(lean, -15.0, 15.0))


def _sine_func(t, amp, freq, phase, offset):
    return amp * np.sin(2 * np.pi * freq * t + phase) + offset


def _lomb_scargle_peak_freq(t: np.ndarray, y: np.ndarray,
                            fmin: float = 0.2, fmax: float = 5.0,
                            n_freqs: int = 2000) -> float:
    """Lomb-Scargle periodogram で不等間隔サンプルから支配周波数を求める。"""
    from scipy.signal import lombscargle
    freqs = np.linspace(fmin, fmax, n_freqs)
    pgram = lombscargle(t, y, 2 * np.pi * freqs, normalize=True)
    return float(freqs[np.argmax(pgram)])


def _sine_r2(t: np.ndarray, y: np.ndarray, freq_guess: float) -> tuple:
    """freq_guess 付近でサイン波をフィットし (popt, r2) を返す。"""
    amp0 = y.std() * np.sqrt(2)
    p0 = [amp0, freq_guess, 0.0, 0.0]
    bounds = ([0,           freq_guess * 0.5, -np.pi, -amp0 * 2],
              [amp0 * 3,    freq_guess * 2.0,  np.pi,  amp0 * 2])
    popt, _ = curve_fit(_sine_func, t, y, p0=p0, bounds=bounds, maxfev=5000)
    residuals = y - _sine_func(t, *popt)
    ss_tot = np.sum(y ** 2)
    r2 = 1.0 - np.sum(residuals ** 2) / ss_tot if ss_tot > 0 else 0.0
    return popt, r2


def est_cycle_duration(rows: list, forward_stride: float, default: float = 1.2) -> float:
    """
    L/R foot_x に Lomb-Scargle + 正弦波フィットして cycle_duration を推定。
    不等間隔サンプリングに対応。フィット品質 R²<0.3 の場合はデフォルト値を返す。
    """
    uniq = unique_by_timestamp(rows)
    ts = np.array([r[COL_TIMESTAMP] for r in uniq]) * FRAME_PERIOD  # seqno → s
    t_rel = ts - ts[0]

    # L と R の foot_x を DC 除去して結合（左右で位相が π ずれるので別々に推定）
    signals = {
        "L": np.array([r[COL_L_FOOT_X] * MM2M for r in uniq]),
        "R": np.array([r[COL_R_FOOT_X] * MM2M for r in uniq]),
    }

    best_r2, best_cycle = -np.inf, default
    for label, sig in signals.items():
        sig = sig - sig.mean()
        if sig.std() < 1e-6:
            continue

        # Lomb-Scargle で支配周波数を探索
        try:
            freq_ls = _lomb_scargle_peak_freq(t_rel, sig)
        except Exception:
            freq_ls = 1.0 / default

        # 支配周波数付近で curve_fit
        try:
            popt, r2 = _sine_r2(t_rel, sig, freq_ls)
            freq = abs(popt[1])
            cycle = float(np.clip(1.0 / freq, 0.2, 4.0))
            print(f"  [cycle_duration/{label}] Lomb-Scargle→{freq_ls:.3f}Hz, "
                  f"fit→{freq:.3f}Hz ({cycle:.3f}s), R²={r2:.2f}")
            if r2 > best_r2:
                best_r2, best_cycle = r2, cycle
        except Exception as e:
            print(f"  [cycle_duration/{label}] フィット失敗 ({e})")

    if best_r2 < 0.3:
        print(f"  [cycle_duration] フィット品質が低い (best R²={best_r2:.2f})。"
              f"デフォルト {default}s を使用。")
        return default
    print(f"  [cycle_duration] 採用: {best_cycle:.3f}s (R²={best_r2:.2f})")
    return best_cycle


def est_swing_ratio(rows: list, foot_lift: float) -> float:
    """
    foot_z > 閾値（foot_lift の 5%）の行の割合から遊脚期間比率を推定。
    左右の平均を返す。
    """
    thr = foot_lift * 0.05
    l_fz = np.array([r[COL_L_FOOT_Z] * MM2M for r in rows])
    r_fz = np.array([r[COL_R_FOOT_Z] * MM2M for r in rows])
    n = len(rows)
    ratio_l = np.sum(l_fz > thr) / n
    ratio_r = np.sum(r_fz > thr) / n
    ratio = float(np.mean([ratio_l, ratio_r]))
    return float(np.clip(ratio, 0.10, 0.70))


# ─────────────────────────────────────────────────────────────────────
# メイン
# ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="前進歩行ログから歩容パラメータを推定する",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--log", type=str, default=None,
                        help="入力 CSV パス (省略時: log/ 内の最新 logs-*.csv)")
    parser.add_argument("--walkparam", type=str, default=WALKPARAM,
                        help="ベース walkparam.json のパス")
    parser.add_argument("--out", type=str, default=None,
                        help="出力 JSON パス (省略時: log/walkparam_est_YYYYMMDDHHMM.json)")
    args = parser.parse_args()

    # 入力ファイル選択
    log_path = args.log if args.log else find_latest_log()
    print(f"入力ログ: {log_path}")

    # ベースパラメータ読み込み
    base_params = {}
    if os.path.exists(args.walkparam):
        with open(args.walkparam, encoding="utf-8") as f:
            base_params = json.load(f)
        print(f"ベースパラメータ: {args.walkparam}")
    else:
        print(f"警告: {args.walkparam} が見つかりません。デフォルト値のみ使用。")

    # CSV 読み込み・フィルタ
    all_rows = load_csv(log_path)
    fwd_rows = filter_forward(all_rows)
    if not fwd_rows:
        print("Error: 前進歩行データが見つかりません（col[17] > 0.1 の行なし）。",
              file=sys.stderr)
        sys.exit(1)

    n_total = len(all_rows)
    n_fwd   = len(fwd_rows)
    ts_arr  = [r[COL_TIMESTAMP] for r in fwd_rows]
    dur_sec = (max(ts_arr) - min(ts_arr)) * FRAME_PERIOD
    print(f"全行数: {n_total}, 前進歩行行数: {n_fwd}, 区間長: {dur_sec:.2f} s")

    # ── パラメータ推定 ─────────────────────────────────────────────
    print("\n── パラメータ推定 ─────────────────────────────")

    forward_stride     = est_forward_stride(fwd_rows)
    foot_lift          = est_foot_lift(fwd_rows)
    hip_swing          = est_hip_swing(fwd_rows)
    forward_lean_angle = est_forward_lean_angle(fwd_rows)
    cycle_default      = base_params.get("cycle_duration", 1.2)
    cycle_duration     = est_cycle_duration(fwd_rows, forward_stride, cycle_default)
    swing_ratio        = est_swing_ratio(fwd_rows, foot_lift)

    print(f"  forward_stride     = {forward_stride:.4f} m")
    print(f"  foot_lift          = {foot_lift:.4f} m")
    print(f"  hip_swing          = {hip_swing:.4f} m")
    print(f"  forward_lean_angle = {forward_lean_angle:.2f} deg")
    print(f"  cycle_duration     = {cycle_duration:.3f} s")
    print(f"  swing_ratio        = {swing_ratio:.3f}")

    # ── 出力 JSON 生成 ────────────────────────────────────────────
    # ベースパラメータにオーバーライド
    out_params = dict(base_params)
    out_params["forward_stride"]      = round(forward_stride,     4)
    out_params["foot_lift"]           = round(foot_lift,          4)
    out_params["hip_swing"]           = round(hip_swing,          4)
    out_params["forward_lean_angle"]  = round(forward_lean_angle, 2)
    out_params["cycle_duration"]      = round(cycle_duration,     3)
    out_params["swing_ratio"]         = round(swing_ratio,        3)

    # 出力パス
    if args.out:
        out_path = args.out
    else:
        ts_str = datetime.now().strftime("%Y%m%d%H%M")
        out_path = os.path.join(LOG_DIR, f"walkparam_est_{ts_str}.json")

    os.makedirs(os.path.dirname(out_path) if os.path.dirname(out_path) else ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out_params, f, indent=2, ensure_ascii=False)
    print(f"\n推定結果を保存: {out_path}")

    # ── ベースとの差分表示 ─────────────────────────────────────────
    if base_params:
        print("\n── ベース (walkparam.json) との差分 ─────────────")
        estimated_keys = ["forward_stride", "foot_lift", "hip_swing",
                          "forward_lean_angle", "cycle_duration", "swing_ratio"]
        for k in estimated_keys:
            base_v = base_params.get(k, "N/A")
            est_v  = out_params[k]
            print(f"  {k:28s}: {base_v!s:>10}  →  {est_v!s}")


if __name__ == "__main__":
    main()
