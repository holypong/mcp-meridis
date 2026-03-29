"""
歩行制御モジュール
ロボットの歩行動作を計算・制御するためのクラスと関数を提供
"""
import numpy as np
import time
import threading
import json
import os
from dataclasses import dataclass, field, asdict

def param_field(default, description, type_):
    return field(default=default, metadata={"description": description, "type": type_})

@dataclass
class WalkParams:
    frame_interval: float = param_field(0.010, "1フレームあたりの時間間隔（固定）[秒] (10ms)", "float")
    phase_offset: float = param_field(np.pi, "左右の足の位相差[rad]", "float")
    init_wait_time: float = param_field(0.0, "初期待機時間[秒]", "float")
    landing_period_ratio: float = param_field(0.10, "両足着地期間の比率 (周期の%、atm_uvc: TERM_FOOT_LAND)", "float")
    weight_shift_duration_ratio: float = param_field(0.25, "重心移動期間の比率 (周期の%)", "float")
    end_of_simulation: float = param_field(8.0, "シミュレーション終了時間[秒]", "float")
    cycle_duration: float = param_field(1.6, "1周期の時間[秒]", "float")
    swing_ratio: float = param_field(0.4, "遊脚期間の比率 (0.0-1.0、推奨0.4)", "float")
    foot_lift: float = param_field(0.014, "遊脚の持ち上げ量[m]", "float")
    hip_swing: float = param_field(0.018, "横方向のスイング量[m]", "float")
    lateral_swing_ratio_1st: float = param_field(0.8, "初期の重心移動時の横スイング倍率", "float")    
    forward_stride: float = param_field(0.02, "前後方向の歩幅[m]", "float")
    duration: float = param_field(8.0, "動作期間[秒]", "float")
    forward_lean_angle: float = param_field(0.0, "歩行中の前傾角度[度]", "float")
    shoulder_roll_angle: float = param_field(10.0, "歩行中の両肩ロール角度[度]", "float")
    smooth_stop: bool = param_field(False, "停止時に自動で一歩追加してその場足踏みするか", "bool")
    mix_enable: bool = param_field(False, "ロール角を足首に反映するか", "bool")
    mix_gyro_g: float = param_field(0.001, "ジャイロミキシングゲイン係数", "float")

@dataclass
class LinkParams:
    HIP_YAW_TO_ROLL_OFFSET: float = param_field(0.07, "ヨー軸とロール軸の間のオフセット[m]", "float")
    THIGH_LENGTH: float = param_field(0.065, "太ももの長さ[m]", "float")
    SHANK_LENGTH: float = param_field(0.065, "すねの長さ[m]", "float")
    ANKLE_LENGTH: float = param_field(0.040, "足首から足首ロールまでの長さ[m]", "float")
    ANKLE_TO_FOOT: float = param_field(0.05, "足首ロール軸から足裏までの距離[m]", "float")
    SHORTEN_LEG_LENGTH: float = param_field(0.02, "短縮時の脚長[m]", "float")

    @property
    def TOTAL_LEG_LENGTH(self):
        return self.THIGH_LENGTH + self.SHANK_LENGTH + self.ANKLE_LENGTH

    @property
    def LINK_LEG_LENGTH(self):
        return self.THIGH_LENGTH + self.SHANK_LENGTH


def load_walk_params(json_path="walkparam-s1.json"):
    """
    JSONファイルからWalkParamsを読み込む
    Args:
        json_path: JSONファイルのパス
    Returns:
        WalkParamsインスタンス
    """
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # デフォルト値でインスタンスを作成してから、JSONの値で上書き
            params = WalkParams()
            for key, value in data.items():
                if hasattr(params, key):
                    setattr(params, key, value)
            print(f"WalkParams loaded from {json_path}")
            return params
        except Exception as e:
            print(f"Error loading {json_path}: {e}. Using default values.")
            return WalkParams()
    else:
        print(f"{json_path} not found. Using default values.")
        return WalkParams()


def save_walk_params(params, json_path="walkparam-s1.json"):
    """
    WalkParamsをJSONファイルに保存
    Args:
        params: WalkParamsインスタンス
        json_path: JSONファイルのパス
    """
    try:
        # dataclassをdictに変換（@propertyは除外）
        data = asdict(params)
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"WalkParams saved to {json_path}")
    except Exception as e:
        print(f"Error saving {json_path}: {e}")


def load_link_params(json_path="linkparam.json"):
    """
    JSONファイルからLinkParamsを読み込む
    Args:
        json_path: JSONファイルのパス
    Returns:
        LinkParamsインスタンス
    """
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # デフォルト値でインスタンスを作成してから、JSONの値で上書き
            params = LinkParams()
            for key, value in data.items():
                if hasattr(params, key):
                    setattr(params, key, value)
            print(f"LinkParams loaded from {json_path}")
            return params
        except Exception as e:
            print(f"Error loading {json_path}: {e}. Using default values.")
            return LinkParams()
    else:
        print(f"{json_path} not found. Using default values.")
        return LinkParams()


def save_link_params(params, json_path="linkparam.json"):
    """
    LinkParamsをJSONファイルに保存
    Args:
        params: LinkParamsインスタンス
        json_path: JSONファイルのパス
    """
    try:
        # dataclassをdictに変換（@propertyは除外）
        data = {k: v for k, v in asdict(params).items() if not k.startswith('_')}
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"LinkParams saved to {json_path}")
    except Exception as e:
        print(f"Error saving {json_path}: {e}")


class WalkController:
    """歩行制御を管理するクラス"""
    
    # ジャイロセンサーのミキシングマトリックス（左）
    MV_MIX_L = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],   # 0 gyro_roll
        [0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],   # 1 gyro_pitch
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],   # 2 gyro_yaw
    ]

    # ジャイロセンサーのミキシングマトリックス（右）
    MV_MIX_R = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0],   # 0 gyro_roll
        [0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],   # 1 gyro_pitch
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],   # 2 gyro_yaw
    ]
    
    # ステータス定数
    IDLE = 0
    WALK = 1
    
    def __init__(self, params=None, params_link=None, msg_size=90):
        """
        Args:
            params: WalkParamsインスタンス（Noneの場合はデフォルト値で初期化）
            params_link: LinkParamsインスタンス（Noneの場合はデフォルト値で初期化）
            msg_size: Meridim配列のサイズ
        """
        self.params = params if params is not None else WalkParams()
        self.params_link = params_link if params_link is not None else LinkParams()
        self.msg_size = msg_size
        
        # 状態変数
        self.mot_sts = self.IDLE
        self.w_sts = 0
        self.t = 0
        self.foot_ref_pitch = np.radians(0.0)
        self.trq_on = 1.0
        self.stop_requested = False  # 安全停止要求フラグ (B2修正)
        self.use_zero_stride = False  # その場足踏みモード（停止準備）
        
        # データバッファ
        self.data = [0.0] * msg_size
        self.buf_output = [[0.0] * msg_size for _ in range(10000)]
        self.buf_input = [[0.0] * msg_size for _ in range(10000)]
        self.buf_index = 0
        
        # スレッド制御
        self.background_thread = None
        self.stop_background = False
        
    def calculate_foot_height(self, phase, step_height):
        """
        UVCスタイルの遊脚高さを計算する関数
        Args:
            phase: 歩行周期の位相 (0-2π)
            step_height: 最大足上げ高さ
        Returns:
            foot_height: 足の高さ
        """
        normalized_phase = ((phase % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
        
        swing_duration = self.params.swing_ratio * 2.0 * np.pi
        swing_start = np.pi - swing_duration / 2
        swing_end = np.pi + swing_duration / 2

        if swing_start <= normalized_phase <= swing_end:
            swing_phase = (normalized_phase - swing_start) / swing_duration
            return step_height * np.sin(swing_phase * np.pi)
        else:
            return 0.0

    def calculate_forward_motion(self, phase, step_length):
        """
        前後方向の移動量を計算する関数
        Args:
            phase: 歩行周期の位相 (0-2π)
            step_length: 歩幅 [m]
        Returns:
            x_pos: 前後方向の位置
        """
        return step_length * np.sin(phase)

    def apply_gyro_mixing(self, data, gyro_values):
        """
        data配列にジャイロミキシングを適用する関数
        Args:
            data: Meridim90配列（関節角度が格納されている）
            gyro_values: ジャイロセンサー値 [gyro_x(roll), gyro_y(pitch), gyro_z(yaw)]
        """
        gyro_x = gyro_values[0]
        gyro_y = gyro_values[1]
        
        mix_gyro_g = self.params.mix_gyro_g  # data[]は度単位のためdeg2rad変換不要

        # 左脚のミキシング
        for i in range(15):
            mix_l = 0.0
            mix_l += gyro_x * float(self.MV_MIX_L[0][i]) * mix_gyro_g
            mix_l += gyro_y * float(self.MV_MIX_L[1][i]) * mix_gyro_g
            data[21 + i*2] += mix_l

        # 右脚のミキシング
        for i in range(15):
            mix_r = 0.0
            mix_r += gyro_x * float(self.MV_MIX_R[0][i]) * mix_gyro_g
            mix_r += gyro_y * float(self.MV_MIX_R[1][i]) * mix_gyro_g
            data[51 + i*2] += mix_r

    def geometric_leg_ik(self, target_pos, target_roll=None, target_pitch=None, is_left=True):
        """
        改善版の幾何学的な逆運動学を解く関数
        Args:
            target_pos: 目標位置 [x, y, z]
            target_roll: 目標ロール角
            target_pitch: 目標ピッチ角
            is_left: 左脚かどうか
        Returns:
            関節角度配列 [hip_yaw, hip_roll, thigh_pitch, knee_pitch, ankle_pitch, ankle_roll]
        """
        leg_length = np.linalg.norm(target_pos)
        if leg_length > self.params_link.TOTAL_LEG_LENGTH:
            print("Leg length is too long. Terminate the simulation.")
            return None

        actual_leg_length = np.sqrt(target_pos[1]**2 + target_pos[2]**2)
        
        # Step 1: hip_rollの計算
        hip_roll = np.arctan2(target_pos[1], actual_leg_length)
        
        MAX_ROLL = np.pi/4
        hip_roll = np.clip(hip_roll, -MAX_ROLL, MAX_ROLL)
        
        # Step 2: 矢状面での解法
        xz_proj_length = np.sqrt(target_pos[0]**2 + target_pos[2]**2)

        cos_knee = (self.params_link.THIGH_LENGTH**2 + self.params_link.SHANK_LENGTH**2 - xz_proj_length**2) / \
                   (2 * self.params_link.THIGH_LENGTH * self.params_link.SHANK_LENGTH)
        knee_pitch = np.pi - np.arccos(np.clip(cos_knee, -1.0, 1.0))
        
        if knee_pitch == 0.0:
            print("Knee pitch is zero. Terminate the simulation.")
            return None

        leg_angle = np.arctan2(target_pos[0], target_pos[2])
        thigh_pitch = leg_angle - knee_pitch / 2.0
        ankle_pitch = -(leg_angle + knee_pitch / 2.0)

        ankle_roll = -hip_roll

        if target_roll is not None:
            hip_roll_correction = target_roll * 0.5
            hip_roll += hip_roll_correction
            ankle_roll -= hip_roll_correction
            
            hip_roll = np.clip(hip_roll, -MAX_ROLL, MAX_ROLL)
            ankle_roll = np.clip(ankle_roll, -MAX_ROLL, MAX_ROLL)

        if target_pitch is not None:
            ankle_pitch += target_pitch

        hip_yaw = 0.0
        
        return np.array([hip_yaw, hip_roll, thigh_pitch, knee_pitch, ankle_pitch, ankle_roll])

    def compute_walking_pose(self, receiver=None, transfer=None, redis_key_read=None, redis_key_write=None):
        """
        1フレーム分の歩行姿勢を計算してデータを更新
        Args:
            receiver: RedisReceiverインスタンス（オプション）
            transfer: RedisTransferインスタンス（オプション）
            redis_key_read: Redis読み込みキー
            redis_key_write: Redis書き込みキー
        Returns:
            (data, get_data): 計算結果のデータと受信データ
        """
        if self.w_sts == 0:
            # 初期位置
            l_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])
            r_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])

            l_joint_angles = self.geometric_leg_ik(l_target_pos, is_left=True)
            r_joint_angles = self.geometric_leg_ik(r_target_pos, is_left=False)

        elif self.w_sts == 1:
            # 両足着地安定期間
            l_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])
            r_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])

            l_joint_angles = self.geometric_leg_ik(l_target_pos, is_left=True)
            r_joint_angles = self.geometric_leg_ik(r_target_pos, is_left=False)

        else:
            # 歩行動作
            phase_y = 2 * np.pi * ((self.t - self.params.init_wait_time) / self.params.cycle_duration)
            phase_z = 2 * np.pi * ((self.t - (self.params.init_wait_time + self.params.cycle_duration * self.params.weight_shift_duration_ratio)) / self.params.cycle_duration)

            if self.w_sts == 2:
                lateral_swing = self.params.hip_swing * np.sin(phase_y) * self.params.lateral_swing_ratio_1st
                l_foot_swing = 0
                r_foot_swing = 0
                l_forward = 0
                r_forward = 0
            else:
                # サイクル開始位相の検出（位相が0～0.3πの範囲にいるか）
                normalized_phase_z = ((phase_z % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
                at_cycle_start = (normalized_phase_z < 0.3 * np.pi)
                
                # 停止要求がある場合、smooth_stop設定に応じて処理
                if self.params.smooth_stop:
                    # smooth_stop有効: サイクル開始位相でその場足踏みモードに移行
                    if self.stop_requested and not self.use_zero_stride and at_cycle_start:
                        self.use_zero_stride = True
                else:
                    # smooth_stop無効: 停止要求があれば即座にその場足踏みモードへ
                    if self.stop_requested and not self.use_zero_stride:
                        self.use_zero_stride = True
                
                lateral_swing = self.params.hip_swing * np.sin(phase_y)
                l_foot_swing = self.calculate_foot_height(phase_z, self.params.foot_lift)
                r_foot_swing = self.calculate_foot_height(phase_z + self.params.phase_offset, self.params.foot_lift)

                # その場足踏みモードではストライド0、通常時は設定値
                if self.use_zero_stride:
                    l_forward = 0.0
                    r_forward = 0.0
                else:
                    l_forward = self.calculate_forward_motion(phase_z, self.params.forward_stride)
                    r_forward = self.calculate_forward_motion(phase_z + self.params.phase_offset, self.params.forward_stride)

            l_target_pos = np.array([l_forward, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])
            r_target_pos = np.array([r_forward, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])

            l_target_pos[2] = l_target_pos[2] - l_foot_swing
            l_target_pos[1] = l_target_pos[1] - lateral_swing
            r_target_pos[2] = r_target_pos[2] - r_foot_swing
            r_target_pos[1] = r_target_pos[1] + lateral_swing

            l_target_roll = 0.0
            r_target_roll = 0.0

            l_joint_angles = self.geometric_leg_ik(l_target_pos, target_pitch=self.foot_ref_pitch,
                                            target_roll=l_target_roll, is_left=True)
            r_joint_angles = self.geometric_leg_ik(r_target_pos, target_pitch=self.foot_ref_pitch,
                                            target_roll=r_target_roll, is_left=False)

        # すべての状態でIK計算結果をdata配列に書き込み (B1修正)
        for i in range(6):
            self.data[30+2*i] = float(self.trq_on)
            self.data[31+2*i] = float(np.degrees(l_joint_angles[i]))
            self.data[60+2*i] = float(self.trq_on)
            self.data[61+2*i] = float(np.degrees(r_joint_angles[i]))

        # 計算した足位置をmeridim90配列にセット（redis_plotter.pyでプロット可能）
        FOOT_POS_DECIMALS = 6
        # 左足位置 (l_foot_x, l_foot_y, l_foot_z) [m]
        self.data[47] = round(float(l_target_pos[0]), FOOT_POS_DECIMALS)  # l_foot_x [m]
        self.data[48] = round(float(l_target_pos[1]), FOOT_POS_DECIMALS)  # l_foot_y [m]
        self.data[49] = round(float(l_target_pos[2]), FOOT_POS_DECIMALS)  # l_foot_z [m]
        # 右足位置 (r_foot_x, r_foot_y, r_foot_z) [m]
        self.data[77] = round(float(r_target_pos[0]), FOOT_POS_DECIMALS)  # r_foot_x [m]
        self.data[78] = round(float(r_target_pos[1]), FOOT_POS_DECIMALS)  # r_foot_y [m]
        self.data[79] = round(float(r_target_pos[2]), FOOT_POS_DECIMALS)  # r_foot_z [m]

        # 歩行中のみ前傾姿勢を適用
        if self.w_sts >= 2 and self.params.forward_lean_angle != 0.0:
            self.data[35] += self.params.forward_lean_angle
            self.data[65] += self.params.forward_lean_angle

        # 歩行中は両肩ロール軸を15度回転
        if self.w_sts >= 1:
            self.data[24] = float(self.trq_on)
            self.data[25] = float(self.params.shoulder_roll_angle)
            self.data[54] = float(self.trq_on)
            self.data[55] = float(self.params.shoulder_roll_angle)

        # Redisからデータを受信（オプション）
        get_data = None
        if receiver is not None and redis_key_read is not None:
            get_data = receiver.get_data(key=redis_key_read)

            if get_data is not None:
                # Redisから読み取ったロボット応答データ（IMU値、ジャイロ値等を含む）を
                # 送信データにコピー（インデックス0-19）
                # これにより、ロボットから受信したセンサ値がそのまま次の送信データに反映される
                for i in range(20):
                    self.data[i] = get_data[i]
                
                imu_roll = get_data[12] if len(get_data) > 12 else 0.0
                imu_pitch = get_data[13] if len(get_data) > 13 else 0.0
                gyro_x = get_data[5] if len(get_data) > 5 else 0.0
                gyro_y = get_data[6] if len(get_data) > 6 else 0.0
                print(f"Time: {self.t:.2f}, State: {self.w_sts}, data[37]: {self.data[37]:.4f}, Roll(X): {imu_roll:.4f}, Pitch(Y): {imu_pitch:.4f}, GyroX: {gyro_x:.4f}, GyroY: {gyro_y:.4f}")
                
                if self.params.mix_enable and len(get_data) > 7:
                    gyro_values = [get_data[5], get_data[6], get_data[7]]
                    self.apply_gyro_mixing(self.data, gyro_values)
        
        # Redisにデータを送信（オプション）
        # self.dataには上記でコピーしたロボット応答データ（IMU値等）と
        # 計算した関節角度指令値が含まれる
        if transfer is not None and redis_key_write is not None:
            transfer.set_data(redis_key_write, self.data)

        # バッファに格納
        if self.w_sts != 0 and self.buf_index < len(self.buf_output):
            self.buf_output[self.buf_index] = self.data.copy()
            self.buf_input[self.buf_index] = get_data.copy() if get_data is not None else [0.0] * self.msg_size
            self.buf_index += 1
        
        return self.data, get_data

    def update_walking_state(self):
        """歩行状態を時間に基づいて更新"""
        if self.t < self.params.init_wait_time:
            self.w_sts = 0
            if self.w_sts == 0:
                self.buf_index = 0
        elif self.t >= self.params.init_wait_time and self.t < (self.params.init_wait_time + self.params.cycle_duration * self.params.landing_period_ratio):
            self.w_sts = 1
        elif self.t >= (self.params.init_wait_time + self.params.cycle_duration * self.params.landing_period_ratio) and self.t < (self.params.init_wait_time + self.params.cycle_duration * self.params.weight_shift_duration_ratio):
            self.w_sts = 2
        elif self.t >= (self.params.init_wait_time + self.params.cycle_duration * self.params.weight_shift_duration_ratio):
            self.w_sts = 3

    def start_walk(self):
        """歩行を開始"""
        self.mot_sts = self.WALK
        self.data[20] = float(self.trq_on)
        self.buf_output = [[0.0] * self.msg_size for _ in range(10000)]
        self.buf_input = [[0.0] * self.msg_size for _ in range(10000)]
        self.buf_index = 0

    def stop_walk(self):
        """歩行を停止"""
        self.mot_sts = self.IDLE
        
        # 初期位置設定
        l_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])
        r_target_pos = np.array([0.0, 0.0, (self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH)])

        l_joint_angles = self.geometric_leg_ik(l_target_pos, is_left=True)
        r_joint_angles = self.geometric_leg_ik(r_target_pos, is_left=False)

        for i in range(6):
            self.data[30+2*i] = float(self.trq_on)
            self.data[31+2*i] = float(np.degrees(l_joint_angles[i]))
            self.data[60+2*i] = float(self.trq_on)
            self.data[61+2*i] = float(np.degrees(r_joint_angles[i]))

        print(f"Time: {self.t:.2f}, State: {self.w_sts} ,data[37]: {self.data[37]}")

    def _current_foot_z(self):
        """現在の足先z（腰基準高さ）を返す。
        data[49]（最終指令値）を優先し、なければ左膝 knee_pitch から幾何推算する。"""
        z = self.data[49]
        if z > 0.001:
            return z
        knee_rad = np.radians(self.data[37])  # 左膝角度 [deg→rad]
        L1 = self.params_link.THIGH_LENGTH
        L2 = self.params_link.SHANK_LENGTH
        z_sq = L1**2 + L2**2 + 2*L1*L2*np.cos(knee_rad)
        if z_sq > 1e-6:
            return np.sqrt(z_sq)
        return self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH

    def _transition_z(self, end_z, transfer, redis_key_write, steps, zero_joints=False):
        """足先zをイーズイン・アウトで補間しながらIKを解いてデータを送信する共通処理。

        Args:
            end_z: 目標とする足先z [m]
            zero_joints: True のとき遷移後に全関節角度をゼロに確定する（Home用）
        """
        self.mot_sts = self.IDLE
        start_z = self._current_foot_z()

        for i in range(6):
            self.data[30+2*i] = float(self.trq_on)
            self.data[60+2*i] = float(self.trq_on)

        for step in range(1, steps + 1):
            alpha = 0.5 * (1.0 - np.cos(np.pi * step / steps))
            z = start_z + alpha * (end_z - start_z)

            l_angles = self.geometric_leg_ik(np.array([0.0, 0.0, z]), is_left=True)
            r_angles = self.geometric_leg_ik(np.array([0.0, 0.0, z]), is_left=False)
            if l_angles is None or r_angles is None:
                continue

            for i in range(6):
                self.data[31+2*i] = float(np.degrees(l_angles[i]))
                self.data[61+2*i] = float(np.degrees(r_angles[i]))

            if transfer is not None and redis_key_write is not None:
                transfer.set_data(redis_key_write, self.data)
            time.sleep(0.010)

        if zero_joints:
            for i in range(30):
                self.data[20+2*i] = float(self.trq_on)
                self.data[21+2*i] = 0.0
            if transfer is not None and redis_key_write is not None:
                transfer.set_data(redis_key_write, self.data)

    def transition_to_stop_walk(self, transfer=None, redis_key_write=None, steps=100):
        """現在位置からIK立位姿勢へ遷移 (Idle用)"""
        end_z = self.params_link.LINK_LEG_LENGTH - self.params_link.SHORTEN_LEG_LENGTH
        self._transition_z(end_z, transfer, redis_key_write, steps)

    def transition_to_reset_pose(self, transfer=None, redis_key_write=None, steps=100):
        """現在位置から全関節ゼロ姿勢へ遷移 (Home用)"""
        end_z = self.params_link.LINK_LEG_LENGTH - 0.001  # 特異点を避ける
        self._transition_z(end_z, transfer, redis_key_write, steps, zero_joints=True)

    def reset_pose(self):
        """姿勢をリセット（全関節をゼロに）"""
        self.mot_sts = self.IDLE
        for i in range(30):
            self.data[20+2*i] = float(self.trq_on)
            self.data[21+2*i] = 0.0
