"""
eval_zmp.py - ZMP評価ライブラリ

FK（順運動学）ベースの ZMP リアルタイム推定ライブラリ。
redis_plotter.py の ZMP 表示モードから呼び出されるほか、
CSV ログのオフライン解析にも単独で使用できる。

依存: numpy のみ（scipy は ConvexHull 精化のオプション）
"""

from __future__ import annotations

import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# scipy.spatial.ConvexHull はオプション（なくても AABB で代替）
try:
    from scipy.spatial import ConvexHull as _ConvexHull
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


# ---------------------------------------------------------------------------
# データクラス
# ---------------------------------------------------------------------------

@dataclass
class LinkParams:
    """ロボットリンクパラメータ（linkparam.json と同じ名称）"""
    HIP_YAW_TO_ROLL_OFFSET: float = 0.07   # 骨盤中心からヒップロール軸までの横オフセット (m)
    THIGH_LENGTH: float = 0.065             # 大腿リンク長 (m)
    SHANK_LENGTH: float = 0.065             # 下腿リンク長 (m)
    ANKLE_LENGTH: float = 0.04             # 足首リンク長 (m)
    ANKLE_TO_FOOT: float = 0.05            # 足首から足裏までの高さ (m)
    SHORTEN_LEG_LENGTH: float = 0.02       # 脚短縮量 (m)

    @property
    def leg_length(self) -> float:
        return self.THIGH_LENGTH + self.SHANK_LENGTH + self.ANKLE_LENGTH


@dataclass
class ZMPResult:
    """ZMP 推定結果"""
    zmp_x: float                        # ZMP X 座標（前後、前方正）(m)
    zmp_y: float                        # ZMP Y 座標（左右、左正）(m)
    com_x: float                        # CoM X 座標 (m)
    com_y: float                        # CoM Y 座標 (m)
    com_z: float                        # CoM Z 座標（高さ）(m)
    support_polygon: np.ndarray         # 支持多角形頂点 shape=(N,2)
    is_stable: bool                     # ZMP が支持多角形内かどうか
    margin: float                       # 余裕量（負なら逸脱）(m)


# ---------------------------------------------------------------------------
# ZMPEstimator
# ---------------------------------------------------------------------------

class ZMPEstimator:
    """
    Meridim90 データ配列からリアルタイムに ZMP を推定するクラス。

    使い方（リアルタイム）:
        est = ZMPEstimator()
        result = est.update(frame_data, t)   # フレームごとに呼ぶ

    使い方（オフライン CSV）:
        import csv, numpy as np
        est = ZMPEstimator()
        with open('buf_output.csv') as f:
            for i, row in enumerate(csv.reader(f)):
                result = est.update([float(v) for v in row], i * 0.01)
    """

    # Meridim90 内のインデックス（redis_plotter.py の joint_to_meridis と対応）
    IDX = {
        'l_hip_yaw':    31, 'l_hip_roll':   33,
        'l_thigh_pitch':35, 'l_knee_pitch': 37,
        'l_ankle_pitch':39, 'l_ankle_roll': 41,
        'l_foot_x':     47, 'l_foot_y':     48, 'l_foot_z': 49,
        'r_hip_yaw':    61, 'r_hip_roll':   63,
        'r_thigh_pitch':65, 'r_knee_pitch': 67,
        'r_ankle_pitch':69, 'r_ankle_roll': 71,
        'r_foot_x':     77, 'r_foot_y':     78, 'r_foot_z': 79,
    }

    # 足裏形状（半サイズ, m）
    FOOT_HALF_LEN   = 0.040   # 前後
    FOOT_HALF_WIDTH = 0.025   # 左右

    # 接地判定しきい値（足先 Z が この値以下なら接地とみなす, m）
    CONTACT_THRESHOLD = 0.006

    # リンク相対質量（等質量モデル、上体が重い）
    _LINK_MASSES = {
        'pelvis':   3.0,
        'l_thigh':  1.0, 'l_shank': 1.0, 'l_foot': 0.5,
        'r_thigh':  1.0, 'r_shank': 1.0, 'r_foot': 0.5,
    }

    def __init__(
        self,
        link_params: Optional[LinkParams] = None,
        dt: float = 0.01,
        history_len: int = 7,
        lp_alpha: float = 0.25,
    ):
        """
        Parameters
        ----------
        link_params : LinkParams, optional
            ロボットリンクパラメータ（省略時はデフォルト値）
        dt : float
            制御周期 (s)。数値微分の参照値として使用
        history_len : int
            CoM 位置の履歴長（数値 2 階微分に使用）
        lp_alpha : float
            加速度ローパスフィルタ係数（0〜1、小さいほど滑らか）
        """
        self.lp = link_params or LinkParams()
        self.dt = dt
        self.lp_alpha = lp_alpha

        # CoM 位置の時刻付き履歴
        self._com_hist: deque[Tuple[float, np.ndarray]] = deque(maxlen=history_len)
        # ローパス済み加速度
        self._com_acc = np.zeros(3)

    # ------------------------------------------------------------------
    # パブリック API
    # ------------------------------------------------------------------

    def update(
        self, frame_data: list, t: float
    ) -> Optional[ZMPResult]:
        """
        1 フレーム分のデータを受け取り ZMP を推定する。

        Parameters
        ----------
        frame_data : list
            Meridim90 配列（長さ 90 以上）
        t : float
            現在時刻 (s)

        Returns
        -------
        ZMPResult | None
            データ不足の場合は None
        """
        if frame_data is None or len(frame_data) < 80:
            return None

        def _get(key: str) -> float:
            idx = self.IDX[key]
            v = frame_data[idx] if idx < len(frame_data) else 0.0
            return float(v) if v is not None else 0.0

        # 足先位置（IK 計算済み値、単位 m）
        lf = np.array([_get('l_foot_x'), _get('l_foot_y'), _get('l_foot_z')])
        rf = np.array([_get('r_foot_x'), _get('r_foot_y'), _get('r_foot_z')])

        # FK でリンク CoM 位置を取得
        link_states = self._fk_link_com(lf, rf)

        # 全体 CoM
        total_mass = sum(m for _, m in link_states)
        com = sum(p * m for p, m in link_states) / total_mass  # type: ignore[arg-type]

        # 加速度推定（数値 2 階微分 + ローパス）
        self._com_hist.append((t, com.copy()))
        com_acc = self._estimate_acceleration()

        # ZMP 計算
        zmp_x, zmp_y = self._compute_zmp(link_states, com_acc)

        # 支持多角形と安定判定
        polygon = self._support_polygon(lf, rf)
        is_stable, margin = self._check_stability(zmp_x, zmp_y, polygon)

        return ZMPResult(
            zmp_x=float(zmp_x),
            zmp_y=float(zmp_y),
            com_x=float(com[0]),
            com_y=float(com[1]),
            com_z=float(com[2]),
            support_polygon=polygon,
            is_stable=is_stable,
            margin=float(margin),
        )

    def reset(self) -> None:
        """履歴をリセット（歩行再開始前などに呼ぶ）"""
        self._com_hist.clear()
        self._com_acc = np.zeros(3)

    # ------------------------------------------------------------------
    # 内部メソッド
    # ------------------------------------------------------------------

    def _fk_link_com(
        self, lf: np.ndarray, rf: np.ndarray
    ) -> List[Tuple[np.ndarray, float]]:
        """
        足先位置からリンク CoM 位置と質量を推定する（簡易 FK）。

        buf_output に格納された IK 計算済み足先位置を起点に
        各リンクの CoM を推定する。
        脚内の各リンク CoM は「隣接関節の中点」で近似。
        """
        lp = self.lp

        # ---- 骨盤（ペルビス）位置の推定 ----
        # 足先 Z の平均 + 脚長 ≒ 骨盤高さ
        foot_avg_z = (lf[2] + rf[2]) / 2
        pelvis_z = foot_avg_z + lp.leg_length
        pelvis_xy = (lf[:2] + rf[:2]) / 2
        pelvis = np.array([pelvis_xy[0], pelvis_xy[1], pelvis_z])

        # ---- ヒップ関節位置 ----
        l_hip = np.array([pelvis[0], pelvis[1] + lp.HIP_YAW_TO_ROLL_OFFSET, pelvis[2]])
        r_hip = np.array([pelvis[0], pelvis[1] - lp.HIP_YAW_TO_ROLL_OFFSET, pelvis[2]])

        # ---- 膝関節位置（足先から上へ）----
        # 足首 → 膝 の位置は足先 Z + ANKLE_LENGTH + SHANK_LENGTH
        l_knee = np.array([lf[0], lf[1], lf[2] + lp.ANKLE_LENGTH + lp.SHANK_LENGTH])
        r_knee = np.array([rf[0], rf[1], rf[2] + lp.ANKLE_LENGTH + lp.SHANK_LENGTH])

        # ---- 足首関節位置 ----
        l_ankle = np.array([lf[0], lf[1], lf[2] + lp.ANKLE_LENGTH])
        r_ankle = np.array([rf[0], rf[1], rf[2] + lp.ANKLE_LENGTH])

        # ---- 各リンク CoM（隣接関節の中点）----
        l_thigh_com = (l_hip + l_knee) / 2
        l_shank_com = (l_knee + l_ankle) / 2
        l_foot_com  = np.array([lf[0], lf[1], lf[2] + lp.ANKLE_LENGTH / 2])

        r_thigh_com = (r_hip + r_knee) / 2
        r_shank_com = (r_knee + r_ankle) / 2
        r_foot_com  = np.array([rf[0], rf[1], rf[2] + lp.ANKLE_LENGTH / 2])

        m = self._LINK_MASSES
        return [
            (pelvis,      m['pelvis']),
            (l_thigh_com, m['l_thigh']),
            (l_shank_com, m['l_shank']),
            (l_foot_com,  m['l_foot']),
            (r_thigh_com, m['r_thigh']),
            (r_shank_com, m['r_shank']),
            (r_foot_com,  m['r_foot']),
        ]

    def _estimate_acceleration(self) -> np.ndarray:
        """
        CoM 位置履歴から数値 2 階微分で加速度を推定し、
        ローパスフィルタをかけて返す。
        """
        if len(self._com_hist) < 3:
            return np.zeros(3)

        hist = list(self._com_hist)
        t2, p2 = hist[-1]
        t1, p1 = hist[-2]
        t0, p0 = hist[-3]

        dt1 = t2 - t1
        dt0 = t1 - t0
        if dt1 < 1e-6 or dt0 < 1e-6:
            return self._com_acc

        # 非等間隔中央差分
        acc_raw = (
            2 * p2 / (dt1 * (dt1 + dt0))
            - 2 * p1 / (dt1 * dt0)
            + 2 * p0 / (dt0 * (dt1 + dt0))
        )

        # ローパスフィルタ（指数移動平均）
        self._com_acc = self.lp_alpha * acc_raw + (1.0 - self.lp_alpha) * self._com_acc
        return self._com_acc

    def _compute_zmp(
        self,
        link_states: List[Tuple[np.ndarray, float]],
        com_acc: np.ndarray,
        g: float = 9.81,
    ) -> Tuple[float, float]:
        """
        ZMP 推定式（Vukobratović 1972）:

            ZMP_x = (Σ m_i (z̈_i + g) x_i − Σ m_i ẍ_i z_i) / Σ m_i (z̈_i + g)

        com_acc は全体 CoM の加速度（スカラーとして全リンクに適用）。
        """
        num_x = num_y = denom = 0.0
        az = float(com_acc[2]) if len(com_acc) > 2 else 0.0
        ax = float(com_acc[0])
        ay = float(com_acc[1])

        for pos, mass in link_states:
            w = mass * (az + g)
            denom += w
            num_x += w * pos[0] - mass * ax * pos[2]
            num_y += w * pos[1] - mass * ay * pos[2]

        if abs(denom) < 1e-9:
            return 0.0, 0.0
        return num_x / denom, num_y / denom

    def _support_polygon(
        self, lf: np.ndarray, rf: np.ndarray
    ) -> np.ndarray:
        """
        足先位置から支持多角形（頂点配列 shape=(N,2)）を返す。

        scipy が利用可能な場合は凸包を計算。
        ない場合は足裏矩形の AABB をそのまま返す。
        """
        fhl = self.FOOT_HALF_LEN
        fhw = self.FOOT_HALF_WIDTH

        def foot_rect(foot_xy: np.ndarray) -> np.ndarray:
            x, y = float(foot_xy[0]), float(foot_xy[1])
            return np.array([
                [x + fhl, y + fhw],
                [x + fhl, y - fhw],
                [x - fhl, y - fhw],
                [x - fhl, y + fhw],
            ])

        l_contact = float(lf[2]) <= self.CONTACT_THRESHOLD
        r_contact = float(rf[2]) <= self.CONTACT_THRESHOLD

        if l_contact and r_contact:
            pts = np.vstack([foot_rect(lf[:2]), foot_rect(rf[:2])])
        elif l_contact:
            pts = foot_rect(lf[:2])
        elif r_contact:
            pts = foot_rect(rf[:2])
        else:
            # フォールバック: 両足中間を接地扱い
            pts = foot_rect((lf[:2] + rf[:2]) / 2)

        if _SCIPY_AVAILABLE and len(pts) >= 4:
            try:
                hull = _ConvexHull(pts)
                return pts[hull.vertices]
            except Exception:
                pass
        return pts

    def _check_stability(
        self, zmp_x: float, zmp_y: float, polygon: np.ndarray
    ) -> Tuple[bool, float]:
        """
        ZMP が支持多角形内にあるか判定する。
        余裕量 = 最近傍辺までの最短距離（内側が正、外側が負）。
        scipy なしの場合は AABB 余裕で近似。
        """
        if _SCIPY_AVAILABLE and len(polygon) >= 3:
            return self._check_stability_polygon(zmp_x, zmp_y, polygon)

        # AABB 近似
        min_x, min_y = polygon.min(axis=0)
        max_x, max_y = polygon.max(axis=0)
        margin = min(
            zmp_x - min_x, max_x - zmp_x,
            zmp_y - min_y, max_y - zmp_y,
        )
        return margin >= 0, float(margin)

    def _check_stability_polygon(
        self, zmp_x: float, zmp_y: float, polygon: np.ndarray
    ) -> Tuple[bool, float]:
        """多角形辺に対する符号付き距離で安定判定（scipy 版）"""
        n = len(polygon)
        p = np.array([zmp_x, zmp_y])
        min_dist = float('inf')
        inside = True

        for i in range(n):
            a = polygon[i]
            b = polygon[(i + 1) % n]
            ab = b - a
            ap = p - a
            ab_len = np.linalg.norm(ab)
            if ab_len < 1e-9:
                continue
            # 辺の法線方向（右手系で外向き）への符号付き距離
            cross = ab[0] * ap[1] - ab[1] * ap[0]
            sign = np.sign(cross)
            dist = abs(cross) / ab_len
            if sign < 0:
                inside = False
                min_dist = -dist if dist < abs(min_dist) else min_dist
            else:
                min_dist = min(min_dist, dist)

        if not inside:
            return False, float(-min(abs(min_dist), 999.0))
        return True, float(min_dist if min_dist != float('inf') else 0.0)
