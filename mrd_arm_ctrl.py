"""
mrd_arm_ctrl.py - ROID1 右腕 IK ライブラリ

モデル:
  waist原点 → 肩オフセット → 肩(pitch + roll) → 上腕 L1 → 肘(yaw + pitch) → 前腕 L2 → 手先

座標系 (MuJoCo/waist frame):
  X: 前方, Y: 左方, Z: 上方

MJCF軸:
  r_shoulder_pitch  : Y軸回り (sp)  負=前に振る, 正=後に振る
  r_arm_upper_roll  : X軸回り (sr)  負=外に上げる(右方向), 正=内側
  r_elbow_yaw       : Z軸回り (ey)  前腕の捻り
  r_arm_lower_pitch : Y軸回り (ep)  負=屈曲, 正=過伸展

Meridim インデックス:
  52: R_SHOULDER_P [deg]
  54: R_SHOULDER_R [deg]
  56: R_ELBOW_Y    [deg]
  58: R_ELBOW_P    [deg]
"""
import math
import json
import numpy as np
from dataclasses import dataclass


# ─────────────────────────────────────────────────────────
# データクラス
# ─────────────────────────────────────────────────────────

@dataclass
class ArmParams:
    shoulder_x: float  # 肩関節 X オフセット [m] from waist
    shoulder_y: float  # 肩関節 Y オフセット [m] (正値; 右腕は内部で -Y 扱い)
    shoulder_z: float  # 肩関節 Z オフセット [m] from waist
    L1: float          # 上腕長 [m] (肩ロール軸 → 肘ピッチ軸)
    L2: float          # 前腕長 [m] (肘ピッチ軸 → 手先)


@dataclass
class ArmAngles:
    """各関節角度 [deg] — Meridim コマンド値と同一符号"""
    shoulder_p: float  # 右肩ピッチ  → idx 52
    shoulder_r: float  # 右肩ロール  → idx 54
    elbow_y:    float  # 右肘ヨー    → idx 56
    elbow_p:    float  # 右肘ピッチ  → idx 58


# 関節可動域 [deg]
ARM_LIMITS = {
    "shoulder_p": (-180.0, 180.0),
    "shoulder_r": (-190.0,  10.0),
    "elbow_y":    ( -90.0,  90.0),
    "elbow_p":    (-140.0,  30.0),
}


# ─────────────────────────────────────────────────────────
# パラメータ読み込み
# ─────────────────────────────────────────────────────────

def load_arm_params(linkparam_path: str) -> ArmParams:
    """linkparam.json から腕パラメータを読み込む"""
    with open(linkparam_path, encoding="utf-8") as f:
        d = json.load(f)
    return ArmParams(
        shoulder_x=d["SHOULDER_OFFSET_X"],
        shoulder_y=d["SHOULDER_OFFSET_Y"],
        shoulder_z=d["SHOULDER_OFFSET_Z"],
        L1=d["UPPER_ARM_LENGTH"],
        L2=d["LOWER_ARM_LENGTH"],
    )


# ─────────────────────────────────────────────────────────
# 補助関数
# ─────────────────────────────────────────────────────────

def _rodrigues(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """ロドリゲスの回転公式で v を axis まわりに angle [rad] 回転"""
    c, s = math.cos(angle), math.sin(angle)
    return v * c + np.cross(axis, v) * s + axis * float(np.dot(axis, v)) * (1.0 - c)


def shoulder_pos_right(params: ArmParams) -> np.ndarray:
    """右肩関節位置 (waist frame) [m]"""
    return np.array([params.shoulder_x, -params.shoulder_y, params.shoulder_z])


# ─────────────────────────────────────────────────────────
# IK
# ─────────────────────────────────────────────────────────

def compute_right_arm_ik(
    target: np.ndarray,
    params: ArmParams,
    elbow_y_deg: float = 0.0,
    elbow_out: bool = True,
) -> ArmAngles:
    """
    右腕 IK: 手先目標位置 → 関節角度

    Parameters
    ----------
    target      : np.ndarray [x, y, z]  waist frame [m]
    params      : ArmParams
    elbow_y_deg : 肘ヨー固定値 [deg] (デフォルト 0)
    elbow_out   : True = 肘を外向き(右方向、デフォルト)
                  False = 肘を下向き

    Returns
    -------
    ArmAngles [deg], 到達不能な場合は可動域端にクランプして返す
    """
    L1, L2 = params.L1, params.L2
    S = shoulder_pos_right(params)

    # 肩から手先へのベクトル
    p = np.asarray(target, dtype=float) - S
    d = float(np.linalg.norm(p))

    # 到達可能範囲にクランプ
    d_max = L1 + L2 - 1e-4
    d_min = abs(L1 - L2) + 1e-4
    d_eff = float(np.clip(d, d_min, d_max))

    # ── 肘ピッチ (law of cosines) ────────────────────────────────────────
    # 2リンク間の内角 β を求め、関節角 ep = β - π に変換
    #   β = π (180°) → ep = 0  (伸展)
    #   β < π       → ep < 0  (屈曲)
    cos_beta = (L1**2 + L2**2 - d_eff**2) / (2.0 * L1 * L2)
    beta = math.acos(float(np.clip(cos_beta, -1.0, 1.0)))
    elbow_pitch_rad = beta - math.pi   # ≤ 0

    # ── 肩での角度 α (肩→手先ベクトルと上腕がなす角) ───────────────────
    cos_alpha = (L1**2 + d_eff**2 - L2**2) / (2.0 * L1 * d_eff)
    alpha = math.acos(float(np.clip(cos_alpha, -1.0, 1.0)))

    # 肩→手先の単位ベクトル
    p_unit = p / d if d > 1e-6 else np.array([0.0, 0.0, -1.0])

    # 肘優先方向
    #   elbow_out=True  → 肘を右外側 (-Y 方向)
    #   elbow_out=False → 肘を下側   (-Z 方向)
    elbow_pref = np.array([0.0, -1.0, 0.0]) if elbow_out else np.array([0.0, 0.0, -1.0])

    # p_unit × elbow_pref → 上腕を振る回転軸
    rot_axis = np.cross(p_unit, elbow_pref)
    rn = float(np.linalg.norm(rot_axis))
    if rn < 1e-6:                          # p_unit と elbow_pref が平行な場合
        rot_axis = np.cross(p_unit, np.array([1.0, 0.0, 0.0]))
        rn = float(np.linalg.norm(rot_axis))
    rot_axis = rot_axis / rn if rn > 1e-6 else np.array([1.0, 0.0, 0.0])

    # 上腕方向 = p_unit を rot_axis まわりに ±α 回転
    sign = 1.0 if elbow_out else -1.0
    u = _rodrigues(p_unit, rot_axis, sign * alpha)

    # ── 肩角度の逆算 ─────────────────────────────────────────────────────
    # ゼロ姿勢 upper_arm_dir = (0, 0, -1) から各回転後:
    #   upper_arm_dir = R_y(sp) · R_x(sr) · (0,0,-1)
    #                 = (-cos(sr)·sin(sp),  sin(sr),  -cos(sr)·cos(sp))
    # → sr = arcsin(uy)
    # → sp = atan2(-ux, -uz)
    ux, uy, uz = float(u[0]), float(u[1]), float(u[2])
    shoulder_roll_rad  = math.asin(float(np.clip(uy, -1.0, 1.0)))
    shoulder_pitch_rad = math.atan2(-ux, -uz)

    angles = ArmAngles(
        shoulder_p=math.degrees(shoulder_pitch_rad),
        shoulder_r=math.degrees(shoulder_roll_rad),
        elbow_y=elbow_y_deg,
        elbow_p=math.degrees(elbow_pitch_rad),
    )
    return clamp_arm_angles(angles)


# ─────────────────────────────────────────────────────────
# FK (検証用)
# ─────────────────────────────────────────────────────────

def compute_right_arm_fk(angles: ArmAngles, params: ArmParams) -> np.ndarray:
    """
    右腕 FK: 関節角度 → 手先位置 (waist frame)
    elbow_yaw は手先位置に影響しないため無視する
    """
    sp = math.radians(angles.shoulder_p)
    sr = math.radians(angles.shoulder_r)
    ep = math.radians(angles.elbow_p)

    # 上腕方向 (waist frame)
    #   R_y(sp) · R_x(sr) · (0, 0, -1)
    u = np.array([
        -math.cos(sr) * math.sin(sp),
         math.sin(sr),
        -math.cos(sr) * math.cos(sp),
    ])

    # 肘ピッチの回転軸 = 上腕フレームのローカル Y 軸
    #   R_y(sp) · R_x(sr) · (0, 1, 0)
    local_y = np.array([
         math.sin(sr) * math.sin(sp),
         math.cos(sr),
         math.sin(sr) * math.cos(sp),
    ])

    # 前腕方向 = u を local_y まわりに ep 回転
    forearm_dir = _rodrigues(u, local_y, ep)

    S = shoulder_pos_right(params)
    return S + params.L1 * u + params.L2 * forearm_dir


# ─────────────────────────────────────────────────────────
# ユーティリティ
# ─────────────────────────────────────────────────────────

def clamp_arm_angles(angles: ArmAngles) -> ArmAngles:
    """各関節角度を可動域にクランプする"""
    def c(v: float, key: str) -> float:
        lo, hi = ARM_LIMITS[key]
        return max(lo, min(hi, v))
    return ArmAngles(
        shoulder_p=c(angles.shoulder_p, "shoulder_p"),
        shoulder_r=c(angles.shoulder_r, "shoulder_r"),
        elbow_y   =c(angles.elbow_y,    "elbow_y"),
        elbow_p   =c(angles.elbow_p,    "elbow_p"),
    )


def arm_angles_to_meridim(angles: ArmAngles) -> dict[str, float]:
    """ArmAngles → Meridim インデックスの辞書"""
    return {
        52: angles.shoulder_p,
        54: angles.shoulder_r,
        56: angles.elbow_y,
        58: angles.elbow_p,
    }


# ─────────────────────────────────────────────────────────
# 簡易テスト (python mrd_arm_ctrl.py で実行)
# ─────────────────────────────────────────────────────────

if __name__ == "__main__":
    import os

    param_path = os.path.join(os.path.dirname(__file__), "linkparam.json")
    params = load_arm_params(param_path)

    print(f"肩位置 (waist frame): {shoulder_pos_right(params)}")
    print(f"L1={params.L1}, L2={params.L2}, max_reach={params.L1 + params.L2:.3f} m\n")

    test_targets = [
        ("正面水平",      np.array([ 0.10, -0.10, 0.065])),
        ("斜め前下",      np.array([ 0.12, -0.12, -0.02])),
        ("真下",          np.array([ 0.016, -0.074, 0.065 - 0.188 + 0.01])),
        ("真横",          np.array([ 0.016, -0.074 - 0.15, 0.065])),
    ]

    for label, target in test_targets:
        angles = compute_right_arm_ik(target, params)
        fk = compute_right_arm_fk(angles, params)
        err = float(np.linalg.norm(fk - target))
        print(f"[{label}]")
        print(f"  target: {target}")
        print(f"  IK  sp={angles.shoulder_p:7.2f}° sr={angles.shoulder_r:7.2f}°"
              f"  ey={angles.elbow_y:6.2f}°  ep={angles.elbow_p:7.2f}°")
        print(f"  FK  {fk}  誤差={err*1000:.2f} mm\n")
