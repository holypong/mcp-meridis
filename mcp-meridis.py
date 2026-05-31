# ヘルプオプションが指定されている場合は、最小限のインポートで即座に終了
import sys
if __name__ == '__main__' and ('-h' in sys.argv or '--help' in sys.argv):
    import argparse
    parser = argparse.ArgumentParser(description='MCP Meridis - Gradio Web Interface for Robot Control')
    parser.add_argument('--redis',
                        default='redis.json',
                        help='Redis configuration JSON file (default: redis.json)')
    parser.add_argument('--walkparam',
                        default='walkparam.json',
                        help='Walk parameter JSON file (default: walkparam.json)')
    parser.print_help()
    sys.exit(0)

# 通常のインポート
import gradio as gr
import numpy as np
import redis_receiver
import redis_transfer
import threading
import time
import math
import csv
import os
import json
import argparse
import dataclasses
from dataclasses import dataclass, field
import re
from mrd_walk_ctrl import WalkController, WalkParams, LinkParams, load_walk_params, load_link_params, save_walk_params, save_link_params
from mrd_info import MeridimKeyParams, get_key_index_text, get_system_info as _get_system_info
from mrd_arm_ctrl import (
    ArmAngles, ArmParams, load_arm_params,
    compute_right_arm_ik, compute_right_arm_fk,
    compute_left_arm_ik, compute_left_arm_fk,
)

_arm_params_cache: ArmParams | None = None

def _get_arm_params() -> ArmParams:
    global _arm_params_cache
    if _arm_params_cache is None:
        _arm_params_cache = load_arm_params("linkparam.json")
    return _arm_params_cache

# 20260103 安定版

@dataclass
class PadAnalog:
    x: float = 0.0
    y: float = 0.0

@dataclass
class PadState:
    btn: int = 0
    analogl: PadAnalog = field(default_factory=PadAnalog)
    analogr: PadAnalog = field(default_factory=PadAnalog)

# 定数
MSG_SIZE = 90               # Meridim配列の長さ
MSG_BUFF = MSG_SIZE * 2     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2     # Meridim配列のエラーフラグの格納場所
MSG_CKSM = MSG_SIZE - 1     # Meridim配列のチェックサムの格納場所
# Redisサーバー設定（デフォルト値、JSONファイルから読み込まれる）
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_KEY_READ = "meridis_sim_pub"
REDIS_KEY_WRITE = "meridis_ai_pub"

# Redisクライアント関連（初期化は後で行う）
receiver = None
transfer = None

def load_redis_config(json_file="redis.json"):
    """Redis設定をJSONファイルから読み込む"""
    global REDIS_HOST, REDIS_PORT, REDIS_KEY_READ, REDIS_KEY_WRITE
    
    try:
        if not os.path.exists(json_file):
            print(f"[Warning] Redis config file '{json_file}' not found. Using default values.")
            return False
        
        with open(json_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # Redisの設定を読み込み
        if 'redis' in config:
            if 'host' in config['redis']:
                REDIS_HOST = config['redis']['host']
            if 'port' in config['redis']:
                REDIS_PORT = config['redis']['port']
        
        # Redisキーの設定を読み込み
        if 'redis_keys' in config:
            if 'read' in config['redis_keys']:
                REDIS_KEY_READ = config['redis_keys']['read']
            if 'write' in config['redis_keys']:
                REDIS_KEY_WRITE = config['redis_keys']['write']
        
        print(f"[Config] Loaded Redis configuration from '{json_file}'")
        print(f"[Config] Redis: {REDIS_HOST}:{REDIS_PORT}")
        print(f"[Config] Redis Keys: Read='{REDIS_KEY_READ}', Write='{REDIS_KEY_WRITE}'")
        return True
        
    except json.JSONDecodeError as e:
        print(f"[Error] Failed to parse JSON file '{json_file}': {e}")
        return False
    except Exception as e:
        print(f"[Error] Failed to load Redis config from '{json_file}': {e}")
        return False

# バックグラウンド処理用の変数
background_thread = None
stop_background = False

TRQ_ON = 1.0            # サーボパワー 0:OFF, 1:ON

# VLA 腕制御オーバーライド
arm_override: list = [0.0, 0.0, 0.0, 0.0]
arm_override_enabled: bool = False
vla_task: str = ""


# MeridimKeyParamsのインデックスと説明を表示する関数
def getmrdkey():
    return get_key_index_text()
# MeridimKeyParams UI は demo ブロック内の gr.Tab で定義する

# パラメータ一括取得
def get_params_text():
    walk_dict = dataclasses.asdict(params)
    link_dict = dataclasses.asdict(params_link)
    lines = ["[WalkParams]"]
    for k, v in walk_dict.items():
        lines.append(f"{k}={v}")
    lines.append("")
    lines.append("[LinkParams]")
    for k, v in link_dict.items():
        lines.append(f"{k}={v}")
    return "\n".join(lines)

# パラメータ一括設定
def set_params_text(text):
    # WalkParams
    walk = {}
    link = {}
    section = None
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if line == '[WalkParams]':
            section = 'walk'
            continue
        if line == '[LinkParams]':
            section = 'link'
            continue
        m = re.match(r'([A-Za-z0-9_]+)\s*=\s*(.+)', line)
        if m:
            k, v = m.group(1), m.group(2)
            try:
                v = eval(v, {"np": np, "True": True, "False": False})
            except Exception:
                pass
            if section == 'walk':
                walk[k] = v
            elif section == 'link':
                link[k] = v
    for k, v in walk.items():
        if hasattr(params, k):
            setattr(params, k, v)
    for k, v in link.items():
        if hasattr(params_link, k):
            setattr(params_link, k, v)
    
    return get_params_text()
# JSONファイルから初期設定を読み込んでテキストで返す
def get_initial_params_text():
    initial_walk = load_walk_params(WALKPARAM_FILE)
    initial_link = load_link_params("linkparam.json")
    walk_dict = dataclasses.asdict(initial_walk)
    link_dict = dataclasses.asdict(initial_link)
    lines = ["[WalkParams]"]
    for k, v in walk_dict.items():
        lines.append(f"{k}={v}")
    lines.append("")
    lines.append("[LinkParams]")
    for k, v in link_dict.items():
        lines.append(f"{k}={v}")
    return "\n".join(lines)

# Params UI は demo ブロック内の gr.Tab で定義する


"""
Claude Desktop
{
  "mcpServers": {
    "gradio-mcp": {
      "command": "npx.cmd",
      "args": [
        "mcp-remote",
        "http://127.0.0.1:7860/gradio_api/mcp/sse"
      ]
    }
  }
}

Cursor
{
  "mcpServers": {
    "mcp-robot": {
      "url": "http://127.0.0.1:7860/gradio_api/mcp/sse"
    }    
  }
}
"""



# ステータス
IDLE = 0
WALK = 1
MOT_STS = IDLE
MOT_INTERVAL = 0.010    # 10ms間隔で記録
TRANSITION_STEPS_IDLE = 100  # IDLEボタン遷移ステップ数 (× 10ms = 秒数)
TRANSITION_STEPS_HOME = 100  # HOMEボタン遷移ステップ数 (× 10ms = 秒数)
w_sts = 0

# 歩行ループ
t = 0
foot_ref_pitch = np.radians(0.0)  # 基準となる足首ピッチ角


# float型のdataの初期化
data = [0.0] * MSG_SIZE

# バッファ変数
buf_output = [[0.0] * MSG_SIZE for _ in range(10000)]  # 送信データのバッファ（10000個のdata配列を格納）
buf_input = [[0.0] * MSG_SIZE for _ in range(10000)]   # 受信データのバッファ（10000個のdata配列を格納）
buf_index = 0  # インクリメンタルカウンタ


# WalkParamsとLinkParamsはwalk_ctrlからインポート
# JSONファイルから読み込み（なければデフォルト値を使用）
# 起動時引数で上書きされる（main()参照）
WALKPARAM_FILE = "walkparam.json"
params = load_walk_params(WALKPARAM_FILE)
#print(f"Loaded WalkParams: cycle_duration={params.cycle_duration}")

# Redis設定とクライアント初期化は main() 関数で行う


# リンク長の定義と脚全体の長さを構造体にまとめる
# LinkParamsはwalk_ctrlからインポート
# JSONファイルから読み込み（なければデフォルト値を使用）
params_link = load_link_params("linkparam.json")
#print(f"Loaded LinkParams: THIGH_LENGTH={params_link.THIGH_LENGTH}, SHANK_LENGTH={params_link.SHANK_LENGTH}")


# パラメータメタデータ取得関数
def get_params_metadata():
    """
    WalkParams/LinkParamsの各パラメータの説明・型情報を辞書で返す
    """
    def extract_meta(cls):
        return {
            field.name: {
                "description": field.metadata.get("description", ""),
                "type": field.metadata.get("type", ""),
                "default": field.default
            }
            for field in dataclasses.fields(cls)
        }
    return {
        "WalkParams": extract_meta(WalkParams),
        "LinkParams": extract_meta(LinkParams)
    }




# 歩行制御関数はwalk_ctrl.WalkControllerに移動
# WalkControllerインスタンスは main() 関数内で初期化


def background_motion_control():
    global MOT_STS, stop_background, data, t, buf_output, buf_input, buf_index, w_sts
    start_time = None
    
    while not stop_background:
        if MOT_STS == WALK:
            if start_time is None:
                start_time = time.time()
            
            # WalkControllerの時刻を更新
            walk_controller.t = time.time() - start_time
            t = walk_controller.t
            
            # 歩行状態を更新
            walk_controller.update_walking_state()
            w_sts = walk_controller.w_sts
            
            # 歩行姿勢を計算
            data, _ = walk_controller.compute_walking_pose(receiver, transfer, REDIS_KEY_READ, REDIS_KEY_WRITE)
            
            # グローバル変数を更新
            buf_output = walk_controller.buf_output
            buf_input = walk_controller.buf_input
            buf_index = walk_controller.buf_index

            # VLA arm override（歩行中も SmolVLA で右腕を制御）
            if arm_override_enabled:
                data[52] = TRQ_ON; data[53] = arm_override[0]
                data[54] = TRQ_ON; data[55] = arm_override[1]
                data[56] = TRQ_ON; data[57] = arm_override[2]
                data[58] = TRQ_ON; data[59] = arm_override[3]
                transfer.set_data(REDIS_KEY_WRITE, data)

            # 安全停止要求チェック (B2修正): その場足踏み経由でサイクル完了時に停止
            if walk_controller.stop_requested:
                can_stop = False
                if t < params.init_wait_time + params.cycle_duration * params.weight_shift_duration_ratio:
                    can_stop = True  # 遊脚前なので即停止可
                else:
                    phase_z = 2 * np.pi * ((t - (params.init_wait_time + params.cycle_duration * params.weight_shift_duration_ratio)) / params.cycle_duration)
                    
                    # smooth_stop設定に応じた停止条件
                    if params.smooth_stop:
                        # smooth_stop有効: サイクル開始付近（0～0.2π）でのみ停止
                        normalized_phase_z = ((phase_z % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
                        can_stop = walk_controller.use_zero_stride and normalized_phase_z < 0.2 * np.pi
                    else:
                        # smooth_stop無効: その場足踏みモードならすぐ停止
                        can_stop = walk_controller.use_zero_stride
                
                if can_stop:
                    walk_controller.stop_requested = False
                    walk_controller.use_zero_stride = False
                    meridian_command("stop", "", "")
                    continue

            # duration指定があれば自動停止
            if params.duration and t >= params.duration:
                if t >= params.init_wait_time:
                    phase_z = 2 * np.pi * ((t - (params.init_wait_time + params.cycle_duration * params.weight_shift_duration_ratio)) / params.cycle_duration)
                    swing_duration = params.swing_ratio * 2.0 * np.pi
                    swing_start = np.pi - swing_duration / 2
                    swing_end = np.pi + swing_duration / 2
                    
                    normalized_phase_l = ((phase_z % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
                    normalized_phase_r = (((phase_z + params.phase_offset) % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
                    
                    l_grounded = not (swing_start <= normalized_phase_l <= swing_end)
                    r_grounded = not (swing_start <= normalized_phase_r <= swing_end)
                    
                    if l_grounded and r_grounded:
                        meridian_command("stop", "", "")
                        continue
                else:
                    meridian_command("stop", "", "")
                    continue
        else:
            start_time = None

        time.sleep(MOT_INTERVAL)

def start_background_thread():
    """バックグラウンドスレッドを開始"""
    global background_thread, stop_background
    
    if background_thread is None or not background_thread.is_alive():
        stop_background = False
        background_thread = threading.Thread(target=background_motion_control, daemon=True)
        background_thread.start()

def stop_background_thread():
    """バックグラウンドスレッドを停止"""
    global stop_background
    stop_background = True

def meridian_command(command, object, value):
    """
    コマンドとオブジェクトを受け取り、そのコマンドのオブジェクトに対する結果を返す

    Args:
        command: コマンド
        object: 操作対象
        value: 操作値
    Returns:
        コマンドのオブジェクトに対する結果

    """
    global MOT_STS, data, buf_output, buf_input, buf_index

    if(command == "walk"):
        walk_controller.start_walk()
        MOT_STS = WALK
        data = walk_controller.data
        buf_output = walk_controller.buf_output
        buf_input = walk_controller.buf_input
        buf_index = walk_controller.buf_index
        
        start_background_thread()
        return f"歩行開始: command: {command}, object: {object}, value: {value}"

    elif(command == "stop"):
        MOT_STS = IDLE
        stop_background_thread()

        walk_controller.stop_walk()
        data = walk_controller.data
        transfer.set_data(REDIS_KEY_WRITE, data)
        return f"停止: command: {command}, object: {object}, value: {value}"

    elif(command == "idle"):
        MOT_STS = IDLE
        stop_background_thread()

        walk_controller.t = 0
        walk_controller.transition_to_stop_walk(transfer, REDIS_KEY_WRITE, steps=TRANSITION_STEPS_IDLE)
        data = walk_controller.data
        return "IDLE: 歩行直前姿勢へ移行しました。"

    elif(command == "home"):
        MOT_STS = IDLE
        stop_background_thread()

        walk_controller.t = 0
        walk_controller.transition_to_reset_pose(transfer, REDIS_KEY_WRITE, steps=TRANSITION_STEPS_HOME)
        data = walk_controller.data
        return "ホーム姿勢へ移行しました（全関節ゼロ）。"

    elif(command == "reset"):
        MOT_STS = IDLE
        stop_background_thread()

        #walk_controller.reset_pose()
        data = walk_controller.data
        transfer.set_data(REDIS_KEY_WRITE, data)
        return f"リセット: command: {command}, object: {object}, value: {value}"

    else:
        return f"未知のコマンド: {command}"

def robot_walk(duration: str = None):
    """ロボット歩行開始"""
    # durationが未指定ならparams.durationを使う。指定があればparams.durationも上書き
    if duration is None or duration == "":
        duration = params.duration
    else:
        try:
            params.duration = float(duration)
        except Exception:
            pass
    return meridian_command("walk", "", duration)

def robot_stop():
    """ロボット停止（歩行中は設定に応じて安全停止）
    
    smooth_stop=True: サイクル開始付近で一歩追加してその場足踏み後に停止
    smooth_stop=False: 即座にその場足踏みに移行して停止（デフォルト）
    """
    if MOT_STS == WALK:
        walk_controller.stop_requested = True
        if params.smooth_stop:
            return "停止処理中... サイクル完了後に停止します。"
        else:
            return "停止処理中... その場足踏み後に停止します。"
    return meridian_command("stop", "", "")

def robot_home():
    """全関節ゼロのホーム姿勢へ移行"""
    return meridian_command("home", "", "")

def robot_idle():
    """IDLEステータスに移行し、歩行直前姿勢（IK立位）をとる"""
    return meridian_command("idle", "", "")


# Resetタブ用のリセット関数
def system_reset():
    """Resetタブからのリセット操作。1回だけdata[0]=5556で送信"""
    global data
    data[0] = 5556
    transfer.set_data(REDIS_KEY_WRITE, data)
    time.sleep(0.01)  # 少し待ってから元に戻す
    data[0] = 0.0  # 送信後は元に戻す
    # 両腕IKを解除: 関節角を0にリセット
    data[53] = 0.0  # R_SHOULDER_P
    data[55] = 0.0  # R_SHOULDER_R
    data[57] = 0.0  # R_ELBOW_Y
    data[59] = 0.0  # R_ELBOW_P
    data[23] = 0.0  # L_SHOULDER_P
    data[25] = 0.0  # L_SHOULDER_R
    data[27] = 0.0  # L_ELBOW_Y
    data[29] = 0.0  # L_ELBOW_P
    transfer.set_data(REDIS_KEY_WRITE, data)
    return "リセット信号（data[0]=5556）を1回送信しました", "", ""

def robot_status():
    """ロボット状態確認（IMU情報含む）"""
    global MOT_STS, t, w_sts, buf_input, buf_index, receiver, REDIS_KEY_READ
    status_map = {IDLE: "停止中", WALK: "歩行中"}
    current_status = status_map.get(MOT_STS, "不明")
    
    # MeridimKeyParamsインスタンス作成
    mrd = MeridimKeyParams()
    
    # 列挙形式で複数行出力
    lines = [
        f"1. 状態: {current_status}",
        f"2. 時間: {t:.2f}秒",
        f"3. 歩行段階: {w_sts}"
    ]
    
    # IMU情報の取得（現在のRedis読み取りキーを優先し、失敗時はbuf_inputを使用）
    latest_data = None
    if receiver is not None:
        latest_data = receiver.get_data(key=REDIS_KEY_READ)
    if latest_data is None and buf_index > 0:
        latest_data = buf_input[buf_index - 1]

    if latest_data is not None:
        
        # 加速度センサ (m/s^2) - 3次元ベクトル表記
        acc_x = latest_data[mrd.MRD_ACC_X]
        acc_y = latest_data[mrd.MRD_ACC_Y]
        acc_z = latest_data[mrd.MRD_ACC_Z]
        lines.append(f"4. 加速度: ({acc_x:.3f}, {acc_y:.3f}, {acc_z:.3f}) m/s²")
        
        # ジャイロセンサ (rad/s) - 3次元ベクトル表記
        gyro_x = latest_data[mrd.MRD_GYRO_X]
        gyro_y = latest_data[mrd.MRD_GYRO_Y]
        gyro_z = latest_data[mrd.MRD_GYRO_Z]
        lines.append(f"5. ジャイロ: ({gyro_x:.4f}, {gyro_y:.4f}, {gyro_z:.4f}) rad/s")
        
        # 姿勢角 (DMP推定値、度) - 3次元ベクトル表記
        roll = latest_data[mrd.MRD_DIR_ROLL]
        pitch = latest_data[mrd.MRD_DIR_PITCH]
        yaw = latest_data[mrd.MRD_DIR_YAW]
        lines.append(f"6. 姿勢角: ({roll:.2f}, {pitch:.2f}, {yaw:.2f}) 度")
        
        # 転倒判定（ロール・ピッチの閾値チェック）
        roll_threshold = 30.0  # 度
        pitch_threshold = 30.0  # 度
        if abs(roll) > roll_threshold or abs(pitch) > pitch_threshold:
            fall_status = "⚠️ 転倒のおそれ"
        else:
            fall_status = "✓ 正常"
        lines.append(f"7. 転倒判定: {fall_status}")
    else:
        lines.append("4. IMUデータ: データなし")
    
    # 20行に満たない場合は空行で埋める
    while len(lines) < 20:
        lines.append("")
    return "\n".join(lines)

# Redisデータ取得関数

REDIS_KEYS = ['meridis_sim_pub', 'meridis_ai_pub', 'meridis_calc_pub', 'meridis_mgr_pub', 'meridis_console_pub']

def set_redis_key_read(key: str) -> str:
    """
    Redisの読み取りキー(REDIS_KEY_READ)を変更する。
    ロボットからの受信データソースが即座に切り替わる。
    Args:
        key: 設定するRedisキー名。有効値: meridis_sim_pub, meridis_ai_pub, meridis_calc_pub, meridis_mgr_pub, meridis_console_pub
    Returns:
        設定結果メッセージ
    """
    global REDIS_KEY_READ
    if key not in REDIS_KEYS:
        return f"error: invalid key '{key}'. valid keys: {', '.join(REDIS_KEYS)}"
    REDIS_KEY_READ = key
    return f"ok: REDIS_KEY_READ = '{key}'"

def get_redis_key_read() -> str:
    """
    現在のRedis読み取りキー(REDIS_KEY_READ)を返す。
    Returns:
        現在設定されているキー名と有効なキー一覧
    """
    return f"REDIS_KEY_READ: '{REDIS_KEY_READ}'\nvalid keys: {', '.join(REDIS_KEYS)}"

def get_redis_data(key: str):
    """指定キーのRedisデータを取得して表示"""
    try:
        client = receiver.redis_client if receiver else None
        if client is None:
            return "error: Redis client not initialized"
        d = client.hgetall(key)
        if not d:
            return f"(no data for key: {key})"
        arr = [float(d[str(i)]) if str(i) in d else None for i in range(len(d))]
        lines = [f"Key: {key}", "---"] + [f"[{i}] {v}" for i, v in enumerate(arr)]
        return "\n".join(lines)
    except Exception as e:
        return f"error: {e}"


def get_pad_data(key: str):
    """指定キーのRedisデータからPADコントローラ値を取得して表示"""
    try:
        client = receiver.redis_client if receiver else None
        if client is None:
            return "error: Redis client not initialized"
        d = client.hgetall(key)
        if not d:
            return f"(no data for key: {key})"
        arr = [float(d[str(i)]) if str(i) in d else 0.0 for i in range(len(d))]
        if len(arr) < 20:
            return f"error: データ不足 ({len(arr)} 要素、最低20必要)"
        pad = PadState(
            btn=int(arr[15]),
            analogl=PadAnalog(x=arr[16], y=arr[17]),
            analogr=PadAnalog(x=arr[18], y=arr[19]),
        )
        lines = [
            f"Key: {key}",
            "---",
            f"pad.btn = {pad.btn}",
            f"pad.analogl.x = {pad.analogl.x:.2f}",
            f"pad.analogl.y = {pad.analogl.y:.2f}",
            f"pad.analogr.x = {pad.analogr.x:.2f}",
            f"pad.analogr.y = {pad.analogr.y:.2f}",
        ]
        return "\n".join(lines)
    except Exception as e:
        return f"error: {e}"


# buf_output/buf_inputデータ取得関数

# buf_outputデータ取得（1行1要素で表示）
def get_buf_output(start: str = "", count: str = "", decimal: str = ""):
    """
    送信バッファ(buf_output)のデータを範囲指定して取得
    buf_outputはロボットへの指令データ（関節角度指令値、サーボコマンドなど）
    Args:
        start: 開始位置（空文字列の場合は0から）
        count: 取得する数（空文字列の場合は最後まで）
        decimal: 小数点以下の桁数（デフォルト4）
    """
    if buf_index == 0:
        return "(no data)"
    
    # 小数点桁数の処理
    try:
        decimal_places = int(decimal) if decimal else 4
    except ValueError:
        decimal_places = 4
    decimal_places = max(0, decimal_places)  # 負の値を防ぐ
    
    # 開始位置の処理
    try:
        start_idx = int(start) if start else 0
    except ValueError:
        start_idx = 0
    
    # 量の処理
    try:
        if count:
            end_idx = start_idx + int(count)
        else:
            end_idx = buf_index
    except ValueError:
        end_idx = buf_index
    
    # 範囲の調整
    start_idx = max(0, min(start_idx, buf_index))
    end_idx = max(start_idx, min(end_idx, buf_index))
    
    lines = []
    for i in range(start_idx, end_idx):
        # 各要素を指定された桁数でフォーマット
        formatted_data = [f"{val:.{decimal_places}f}" for val in buf_output[i]]
        lines.append(f"[{i}] {formatted_data}")
    
    if not lines:
        return "(no data in specified range)"
    return "\n".join(lines)

# buf_inputデータ取得（1行1要素で表示）
def get_buf_input(start: str = "", count: str = "", decimal: str = "", key: str = ""):
    """
    受信バッファ(buf_input)のデータを範囲指定して取得
    buf_inputはロボットからの応答データ（IMUセンサー値、モーター実測値など）
    Args:
        start: 開始位置（空文字列の場合は0から）
        count: 取得する数（空文字列の場合は最後まで）
        decimal: 小数点以下の桁数（デフォルト4）
        key: 表示対象のRedisキー（情報表示用）
    """
    if buf_index == 0:
        return "(no data)"
    
    # 小数点桁数の処理
    try:
        decimal_places = int(decimal) if decimal else 4
    except ValueError:
        decimal_places = 4
    decimal_places = max(0, decimal_places)  # 負の値を防ぐ
    
    # 開始位置の処理
    try:
        start_idx = int(start) if start else 0
    except ValueError:
        start_idx = 0
    
    # 量の処理
    try:
        if count:
            end_idx = start_idx + int(count)
        else:
            end_idx = buf_index
    except ValueError:
        end_idx = buf_index
    
    # 範囲の調整
    start_idx = max(0, min(start_idx, buf_index))
    end_idx = max(start_idx, min(end_idx, buf_index))
    
    display_key = key if key else REDIS_KEY_READ
    lines = [f"Key: {display_key}", "---"]
    for i in range(start_idx, end_idx):
        formatted_data = [f"{val:.{decimal_places}f}" for val in buf_input[i]]
        lines.append(f"[{i}] {formatted_data}")

    if len(lines) == 2:
        return "(no data in specified range)"
    return "\n".join(lines)


# CSV保存・読み込み関数

def filesave_buf_input():
    """
    buf_input(ロボットからの応答データ)をCSVファイルに保存（固定ファイル名で上書き）
    Returns:
        保存結果のメッセージ
    """
    if buf_index == 0:
        return "エラー: 保存するデータがありません"
    
    try:
        # 固定ファイル名
        filename = "buf_input.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 生データのみ書き込み（ヘッダーなし、インデックスなし）
            for i in range(buf_index):
                writer.writerow(buf_input[i])
        
        abs_path = os.path.abspath(filename)
        return f"保存完了: {filename}\nパス: {abs_path}\nデータ数: {buf_index}"
    except Exception as e:
        return f"エラー: {str(e)}"

def filesave_buf_output():
    """
    buf_output(ロボットへの指令データ)をCSVファイルに保存（固定ファイル名で上書き）
    Returns:
        保存結果のメッセージ
    """
    if buf_index == 0:
        return "エラー: 保存するデータがありません"
    
    try:
        # 固定ファイル名
        filename = "buf_output.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 生データのみ書き込み（ヘッダーなし、インデックスなし）
            for i in range(buf_index):
                writer.writerow(buf_output[i])
        
        abs_path = os.path.abspath(filename)
        return f"保存完了: {filename}\nパス: {abs_path}\nデータ数: {buf_index}"
    except Exception as e:
        return f"エラー: {str(e)}"

def filepathget_buf_input():
    """
    buf_input(ロボットからの応答データ) CSVファイルのパスを返す（AIエージェント用）
    AIエージェントはこのパスを使ってファイルを直接読み取ることができます
    Returns:
        絶対パスまたはエラーメッセージ
    """
    try:
        # 固定ファイル名のbuf_input.csvを使用
        filename = "buf_input.csv"
        
        if os.path.exists(filename):
            return os.path.abspath(filename)
        else:
            return "エラー: buf_input.csvが見つかりません。先にCSV保存を実行してください。"
    except Exception as e:
        return f"エラー: {str(e)}"

def filepathget_buf_output():
    """
    buf_output(ロボットへの指令データ) CSVファイルのパスを返す（AIエージェント用）
    AIエージェントはこのパスを使ってファイルを直接読み取ることができます
    Returns:
        絶対パスまたはエラーメッセージ
    """
    try:
        # 固定ファイル名のbuf_output.csvを使用
        filename = "buf_output.csv"
        
        if os.path.exists(filename):
            return os.path.abspath(filename)
        else:
            return "エラー: buf_output.csvが見つかりません。先にCSV保存を実行してください。"
    except Exception as e:
        return f"エラー: {str(e)}"

def parse_arguments():
    """コマンドライン引数を解析する"""
    parser = argparse.ArgumentParser(description='MCP Meridis - Gradio Web Interface for Robot Control')
    parser.add_argument('--redis',
                        default='redis.json',
                        help='Redis configuration JSON file (default: redis.json)')
    parser.add_argument('--walkparam',
                        default='walkparam.json',
                        help='Walk parameter JSON file (default: walkparam.json)')
    return parser.parse_args()


def get_system_info():
    """システム情報（キーインデックスとパラメータ）を一括取得"""
    return _get_system_info(get_params_text())


# ─────────────────────────────────────────────────────────
# 右腕 IK 制御
# ─────────────────────────────────────────────────────────

def _read_latest_meridim():
    """最新の Meridim データを取得 (buf_input 優先、なければ Redis 直接読み込み)"""
    if buf_index > 0:
        return buf_input[buf_index - 1]
    client = receiver.redis_client if receiver else None
    if client is None:
        return None
    d = client.hgetall(REDIS_KEY_READ)
    if not d:
        return None
    return [float(d[str(i)]) if str(i) in d else 0.0 for i in range(MSG_SIZE)]


def arm_get_state():
    """両腕の現在状態を読み取り: 手先 XYZ (FK) と関節角度を返す"""
    try:
        arm_params = _get_arm_params()
    except Exception as e:
        return f"パラメータエラー: {e}", "", "", ""

    latest = _read_latest_meridim()
    if latest is None:
        return "データなし (Redis 未接続 or バッファ空)", "", "", ""

    r_sp = latest[53]  # R_SHOULDER_P_VAL
    r_sr = latest[55]  # R_SHOULDER_R_VAL
    r_ey = latest[57]  # R_ELBOW_Y_VAL
    r_ep = latest[59]  # R_ELBOW_P_VAL
    r_angles = ArmAngles(shoulder_p=r_sp, shoulder_r=r_sr, elbow_y=r_ey, elbow_p=r_ep)
    r_fk = compute_right_arm_fk(r_angles, arm_params)

    l_sp = latest[23]  # L_SHOULDER_P_VAL
    l_sr = latest[25]  # L_SHOULDER_R_VAL
    l_ey = latest[27]  # L_ELBOW_Y_VAL
    l_ep = latest[29]  # L_ELBOW_P_VAL
    l_angles = ArmAngles(shoulder_p=l_sp, shoulder_r=l_sr, elbow_y=l_ey, elbow_p=l_ep)
    l_fk = compute_left_arm_fk(l_angles, arm_params)

    r_lines = [
        "右腕 関節角度:",
        f"  ID53:肩P = {r_sp:.2f}°",
        f"  ID55:肩R = {r_sr:.2f}°",
        f"  ID57:肘Y = {r_ey:.2f}°",
        f"  ID59:肘P = {r_ep:.2f}°",
    ]
    l_lines = [
        "左腕 関節角度:",
        f"  ID23:肩P = {l_sp:.2f}°",
        f"  ID25:肩R = {l_sr:.2f}°",
        f"  ID27:肘Y = {l_ey:.2f}°",
        f"  ID29:肘P = {l_ep:.2f}°",
    ]
    return (
        "\n".join(r_lines),
        "\n".join(l_lines),
        f"{r_fk[0]:.4f},{r_fk[1]:.4f},{r_fk[2]:.4f}",
        f"{l_fk[0]:.4f},{l_fk[1]:.4f},{l_fk[2]:.4f}",
    )


_ARM_SINGULARITY_THRESHOLD = 8.0  # 肘P がこの角度 [deg] 未満なら特異点付近とみなす

# 特異点エスケープ用の準備ポーズ (肘を90°屈曲した安全姿勢)
ARM_PREP_POSE = ArmAngles(
    shoulder_p=0.0,
    shoulder_r=0.0,
    elbow_y=0.0,
    elbow_p=-90.0,
)


def _arm_send_angles(angles):
    global data
    for cmd_idx, angle in zip(
        (52, 54, 56, 58),
        (angles.shoulder_p, angles.shoulder_r, angles.elbow_y, angles.elbow_p),
    ):
        data[cmd_idx] = TRQ_ON
        data[cmd_idx + 1] = angle
    transfer.set_data(REDIS_KEY_WRITE, data)


def _left_arm_send_angles(angles):
    global data
    for cmd_idx, angle in zip(
        (22, 24, 26, 28),
        (angles.shoulder_p, angles.shoulder_r, angles.elbow_y, angles.elbow_p),
    ):
        data[cmd_idx] = TRQ_ON
        data[cmd_idx + 1] = angle
    transfer.set_data(REDIS_KEY_WRITE, data)


LEFT_ARM_PREP_POSE = ArmAngles(
    shoulder_p=0.0,
    shoulder_r=0.0,
    elbow_y=0.0,
    elbow_p=-90.0,
)


def arm_prep_pose():
    """右腕を準備ポーズ (肘90°屈曲) へ移動する"""
    _arm_send_angles(ARM_PREP_POSE)
    return (
        f"右腕 準備ポーズを送信:\n"
        f"  肩P = {ARM_PREP_POSE.shoulder_p:.1f}°\n"
        f"  肩R = {ARM_PREP_POSE.shoulder_r:.1f}°\n"
        f"  肘Y = {ARM_PREP_POSE.elbow_y:.1f}°\n"
        f"  肘P = {ARM_PREP_POSE.elbow_p:.1f}°"
    )


def left_arm_prep_pose():
    """左腕を準備ポーズ (肘90°屈曲) へ移動する"""
    _left_arm_send_angles(LEFT_ARM_PREP_POSE)
    return (
        f"左腕 準備ポーズを送信:\n"
        f"  肩P = {LEFT_ARM_PREP_POSE.shoulder_p:.1f}°\n"
        f"  肩R = {LEFT_ARM_PREP_POSE.shoulder_r:.1f}°\n"
        f"  肘Y = {LEFT_ARM_PREP_POSE.elbow_y:.1f}°\n"
        f"  肘P = {LEFT_ARM_PREP_POSE.elbow_p:.1f}°"
    )


def arm_set_position(xyz_str):
    """指定 XYZ [m] へ右手を近づける: IK で関節角を計算して送信"""
    global data
    try:
        parts = [s.strip() for s in xyz_str.split(",")]
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    except (ValueError, TypeError, IndexError):
        return "x,y,z の形式で入力してください（例: 0.10,-0.10,0.065）"

    try:
        arm_params = _get_arm_params()
    except Exception as e:
        return f"パラメータエラー: {e}"

    target = np.array([x, y, z])

    # 特異点チェック: 現在の肘角度が閾値未満なら準備ポーズを先送信する
    current_elbow_p = data[59]
    escape_note = None
    if abs(current_elbow_p) < _ARM_SINGULARITY_THRESHOLD:
        _arm_send_angles(ARM_PREP_POSE)
        escape_note = f"[特異点エスケープ] 肘P={current_elbow_p:.1f}° → 準備ポーズを先送信"

    angles = compute_right_arm_ik(target, arm_params)
    _arm_send_angles(angles)

    fk = compute_right_arm_fk(angles, arm_params)
    err_mm = float(np.linalg.norm(fk - target))

    lines = []
    if escape_note:
        lines.append(escape_note)
    lines += [
        "IK 計算結果:",
        f"  肩P = {angles.shoulder_p:.2f}°",
        f"  肩R = {angles.shoulder_r:.2f}°",
        f"  肘Y = {angles.elbow_y:.2f}°",
        f"  肘P = {angles.elbow_p:.2f}°",
        "",
        f"FK 検証 (誤差 {err_mm:.4f} m):",
        f"  X = {fk[0]:.4f} m",
        f"  Y = {fk[1]:.4f} m",
        f"  Z = {fk[2]:.4f} m",
        "",
        "送信完了",
    ]
    return "\n".join(lines)


def left_arm_set_position(xyz_str):
    """指定 XYZ [m] へ左手を近づける: IK で関節角を計算して送信"""
    global data
    try:
        parts = [s.strip() for s in xyz_str.split(",")]
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    except (ValueError, TypeError, IndexError):
        return "x,y,z の形式で入力してください（例: 0.10,0.10,0.065）"

    try:
        arm_params = _get_arm_params()
    except Exception as e:
        return f"パラメータエラー: {e}"

    target = np.array([x, y, z])

    current_elbow_p = data[29]  # L_ELBOW_P_VAL
    escape_note = None
    if abs(current_elbow_p) < _ARM_SINGULARITY_THRESHOLD:
        _left_arm_send_angles(LEFT_ARM_PREP_POSE)
        escape_note = f"[特異点エスケープ] 肘P={current_elbow_p:.1f}° → 準備ポーズを先送信"

    angles = compute_left_arm_ik(target, arm_params)
    _left_arm_send_angles(angles)

    fk = compute_left_arm_fk(angles, arm_params)
    err_mm = float(np.linalg.norm(fk - target))

    lines = []
    if escape_note:
        lines.append(escape_note)
    lines += [
        "IK 計算結果 (左腕):",
        f"  肩P = {angles.shoulder_p:.2f}°",
        f"  肩R = {angles.shoulder_r:.2f}°",
        f"  肘Y = {angles.elbow_y:.2f}°",
        f"  肘P = {angles.elbow_p:.2f}°",
        "",
        f"FK 検証 (誤差 {err_mm:.4f} m):",
        f"  X = {fk[0]:.4f} m",
        f"  Y = {fk[1]:.4f} m",
        f"  Z = {fk[2]:.4f} m",
        "",
        "送信完了",
    ]
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────
# VLA 連携ツール
# ─────────────────────────────────────────────────────────

def set_vla_task(task: str) -> str:
    """VLA 言語タスクを設定する（Claude Code チャットまたは vla_arm_bridge から呼び出す）
    例: "Touch the red ball with your right hand"
    """
    global vla_task
    vla_task = task
    return f"VLA task set: {task}"


def get_vla_task() -> str:
    """現在の VLA タスク文字列を返す（vla_arm_bridge がポーリング）"""
    return vla_task


def arm_override_off() -> str:
    """右腕の Override を無効化する"""
    return set_arm_cmd("[]")


def set_arm_cmd(values_str: str) -> str:
    """右腕コマンドを上書きする [肩P, 肩R, 肘Y, 肘P] (deg), JSON 配列形式。
    空配列 [] で override を無効化。
    例: "[0.0, 0.0, -90.0, 0.0]"
    """
    global arm_override, arm_override_enabled, data
    try:
        vals = json.loads(values_str)
    except (json.JSONDecodeError, TypeError):
        return "エラー: JSON 配列形式で入力してください (例: [0, 0, -90, 0])"

    if isinstance(vals, list) and len(vals) == 4:
        arm_override = [float(v) for v in vals]
        arm_override_enabled = True
    elif isinstance(vals, list) and len(vals) == 0:
        arm_override_enabled = False
    else:
        return "エラー: 4 要素のリスト、または空リスト [] で無効化してください"

    if arm_override_enabled:
        data[52] = TRQ_ON; data[53] = arm_override[0]
        data[54] = TRQ_ON; data[55] = arm_override[1]
        data[56] = TRQ_ON; data[57] = arm_override[2]
        data[58] = TRQ_ON; data[59] = arm_override[3]
        if transfer:
            transfer.set_data(REDIS_KEY_WRITE, data)

    return f"arm_cmd={'on' if arm_override_enabled else 'off'}: {arm_override}"


def main():
    """メイン関数 - コマンドライン引数を処理してGradioアプリを起動"""
    global receiver, transfer, walk_controller, WALKPARAM_FILE, params

    try:
        # コマンドライン引数を解析
        args = parse_arguments()

        # walkparamファイルを引数で上書き
        if args.walkparam != WALKPARAM_FILE:
            WALKPARAM_FILE = args.walkparam
            params = load_walk_params(WALKPARAM_FILE)
            print(f"[Config] WalkParams loaded from '{WALKPARAM_FILE}'")

        # Redis設定をJSONファイルから読み込み
        load_redis_config(args.redis)
        
        # Redisクライアントを初期化
        receiver = redis_receiver.RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)
        transfer = redis_transfer.RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)
        
        # WalkControllerインスタンスを作成
        walk_controller = WalkController(params=params, params_link=params_link, msg_size=MSG_SIZE)
        start_background_thread()

        print(f"[Info] Starting Gradio web interface...")
        #print(f"[Info] Redis config loaded from: {args.redis}")

        # メインUI（walkparamファイル名が確定してから定義）
        with gr.Blocks() as demo:
            with gr.Tabs():
                with gr.Tab("Control"):
                    gr.Markdown("### Control")
                    with gr.Row():
                        home_btn       = gr.Button("Home")
                        idle_btn       = gr.Button("Idle")
                        walk_btn       = gr.Button("Walk")
                        stop_btn       = gr.Button("Stop")
                        reset_btn      = gr.Button("Sysreset")
                        status_btn     = gr.Button("Status")
                        duration_input = gr.Textbox(label="Duration", placeholder="歩行時間（秒）", scale=2)
                    result_out = gr.Textbox(label="Result / Status", lines=20)
                    home_btn.click(fn=robot_home, inputs=[], outputs=result_out)
                    idle_btn.click(fn=robot_idle, inputs=[], outputs=result_out)
                    walk_btn.click(fn=robot_walk, inputs=duration_input, outputs=result_out)
                    stop_btn.click(fn=robot_stop, inputs=[], outputs=result_out)
                    # reset_btn.click は arm_xyz 定義後に記述
                    status_btn.click(fn=robot_status, inputs=[], outputs=result_out)

                with gr.Tab("Params"):
                    gr.Markdown(f"""### パラメータ一括取得・一括設定
1. [メモリを取得]ボタンで現在のメモリ上の値をテキストボックスに表示
2. 編集後、[メモリを設定]ボタンで一括反映
3. [初期設定を取得]で JSON ファイルの初期値を表示（反映するには[メモリを設定]を押す）
4. 初期設定の読み込み元: `{WALKPARAM_FILE}`
""")
                    with gr.Row():
                        get_btn  = gr.Button("メモリを取得")
                        set_btn  = gr.Button("メモリを設定")
                        init_btn = gr.Button("初期設定を取得")
                    param_box = gr.Textbox(label="Params", lines=20)
                    get_btn.click(fn=get_params_text, inputs=[], outputs=param_box)
                    set_btn.click(fn=set_params_text, inputs=param_box, outputs=param_box)
                    init_btn.click(fn=get_initial_params_text, inputs=[], outputs=param_box)

                with gr.Tab("Redis") as redis_tab:
                    gr.Markdown("### Redisデータログ\n対象キーを選択して取得します。")
                    with gr.Row():
                        redis_key_dropdown = gr.Dropdown(choices=REDIS_KEYS, label="Key", value=REDIS_KEY_READ)
                        redis_get_btn = gr.Button("取得")
                        redis_pad_btn = gr.Button("PAD取得")
                    redis_log_box = gr.Textbox(label="Data", lines=20, elem_id="redis_log_box")
                    redis_get_btn.click(fn=get_redis_data, inputs=redis_key_dropdown, outputs=redis_log_box).then(
                        fn=None,
                        js="() => { const el = document.querySelector('#redis_log_box textarea'); if(el) el.scrollTop = 0; }"
                    )
                    redis_pad_btn.click(fn=get_pad_data, inputs=redis_key_dropdown, outputs=redis_log_box)

                with gr.Tab("InputBuf") as inputbuf_tab:
                    gr.Markdown("""### Input Buffer (ロボットからの応答)\nbuf_inputはロボットからの応答データ（IMUセンサー値、モーター実測値など）を格納します。""")
                    with gr.Row():
                        inputbuf_key_dropdown = gr.Dropdown(choices=REDIS_KEYS, label="Key", value=REDIS_KEY_READ)
                        inputbuf_key_get_btn  = gr.Button("現在のキーを確認", scale=1)
                    inputbuf_key_status = gr.Textbox(label="キー設定結果", lines=2, interactive=False)
                    inputbuf_key_dropdown.change(fn=set_redis_key_read, inputs=inputbuf_key_dropdown, outputs=inputbuf_key_status)
                    inputbuf_key_get_btn.click(fn=get_redis_key_read, inputs=[], outputs=inputbuf_key_status)
                    btn_store_csv_in = gr.Button("buf_inputをCSVに保存")
                    csv_store_result_in = gr.Textbox(label="保存結果", lines=3)
                    btn_store_csv_in.click(fn=filesave_buf_input, inputs=[], outputs=csv_store_result_in)
                    gr.Markdown("---")
                    with gr.Row():
                        start_read   = gr.Textbox(label="開始位置", placeholder="0")
                        count_read   = gr.Textbox(label="量", placeholder="最後まで")
                        decimal_read = gr.Textbox(label="小数点桁数", placeholder="4")
                    btn_read = gr.Button("buf_input取得")
                    buf_box_read = gr.Textbox(label="buf_input (受信)", lines=20)
                    btn_read.click(fn=get_buf_input, inputs=[start_read, count_read, decimal_read, inputbuf_key_dropdown], outputs=buf_box_read)

                with gr.Tab("OutputBuf"):
                    gr.Markdown("""### Output Buffer (ロボットへの指令)\nbuf_outputはロボットへの指令データ（関節角度指令値、サーボコマンドなど）を格納します。""")
                    btn_store_csv_out = gr.Button("buf_outputをCSVに保存")
                    csv_store_result_out = gr.Textbox(label="保存結果", lines=3)
                    btn_store_csv_out.click(fn=filesave_buf_output, inputs=[], outputs=csv_store_result_out)
                    gr.Markdown("---")
                    with gr.Row():
                        start_write   = gr.Textbox(label="開始位置", placeholder="0")
                        count_write   = gr.Textbox(label="量", placeholder="最後まで")
                        decimal_write = gr.Textbox(label="小数点桁数", placeholder="4")
                    btn_write = gr.Button("buf_output取得")
                    buf_box_write = gr.Textbox(label="buf_output (送信)", lines=20)
                    btn_write.click(fn=get_buf_output, inputs=[start_write, count_write, decimal_write], outputs=buf_box_write)

                with gr.Tab("GetKeyIndex"):
                    gr.Markdown("""### Meridim90 キーインデックス一覧\n各キーのインデックスと説明を表示します。""")
                    key_btn = gr.Button("一覧取得")
                    key_box = gr.Textbox(label="MeridimKeyParams", lines=30)
                    key_btn.click(fn=getmrdkey, inputs=[], outputs=key_box)

                with gr.Tab("SysInfo"):
                    gr.Markdown("""### システム情報
このタブでは、Meridim90のキーインデックス、歩行パラメータ、リンクパラメータ、使用可能なスキルの一覧を一括で取得できます。
AIエージェントはこの情報を使ってシステムを理解します。""")
                    sysinfo_btn = gr.Button("情報取得")
                    sysinfo_box = gr.Textbox(label="System Info", lines=50)
                    sysinfo_btn.click(fn=get_system_info, inputs=[], outputs=sysinfo_box)

                with gr.Tab("Arm"):
                    gr.Markdown("### 腕 IK 制御（3自由度）\n目標手先位置 [m]  を x,y,z 形式で入力して「設定」すると関節が動きます。 \n「取得」で両腕の手先位置と関節角を読み込みます。")
                    arm_get_btn = gr.Button("取得")
                    with gr.Row():
                        with gr.Column():
                            gr.Markdown("#### 右腕")
                            with gr.Row():
                                arm_set_btn  = gr.Button("設定")
                                arm_prep_btn = gr.Button("準備ポーズ（肘90度）", variant="secondary")
                            r_arm_xyz = gr.Textbox(label="右手 X,Y,Z [m]（腰中心から）", placeholder="0.10,-0.10,0.065")
                            r_arm_result = gr.Textbox(label="結果（右腕）", lines=8)
                        with gr.Column():
                            gr.Markdown("#### 左腕")
                            with gr.Row():
                                l_arm_set_btn  = gr.Button("設定")
                                l_arm_prep_btn = gr.Button("準備ポーズ（肘90度）", variant="secondary")
                            l_arm_xyz = gr.Textbox(label="左手 X,Y,Z [m]（腰中心から）", placeholder="0.10,0.10,0.065")
                            l_arm_result = gr.Textbox(label="結果（左腕）", lines=8)
                    arm_get_btn.click(fn=arm_get_state, inputs=[], outputs=[r_arm_result, l_arm_result, r_arm_xyz, l_arm_xyz])
                    arm_set_btn.click(fn=arm_set_position, inputs=[r_arm_xyz], outputs=r_arm_result)
                    arm_prep_btn.click(fn=arm_prep_pose, inputs=[], outputs=r_arm_result)
                    l_arm_set_btn.click(fn=left_arm_set_position, inputs=[l_arm_xyz], outputs=l_arm_result)
                    l_arm_prep_btn.click(fn=left_arm_prep_pose, inputs=[], outputs=l_arm_result)
                    reset_btn.click(fn=system_reset, inputs=[], outputs=[result_out, r_arm_xyz, l_arm_xyz])

                with gr.Tab("VLA"):
                    gr.Markdown("### VLA 右腕制御\n`vla_arm_bridge.py` 連携ツール。タスク設定と右腕コマンド上書きを管理します。")
                    with gr.Row():
                        vla_task_in = gr.Textbox(label="タスク文字列", placeholder="Touch the red ball with your right hand", scale=3)
                        vla_set_btn = gr.Button("タスク設定")
                        vla_get_btn = gr.Button("タスク取得")
                    vla_task_box = gr.Textbox(label="現在のタスク", lines=2)
                    gr.Markdown("---")
                    with gr.Row():
                        arm_cmd_in  = gr.Textbox(label="腕コマンド JSON [肩P, 肩R, 肘Y, 肘P] deg", placeholder="[0.0, 0.0, -90.0, 0.0]", scale=3)
                        arm_cmd_btn = gr.Button("送信")
                        arm_off_btn = gr.Button("Override OFF", variant="secondary")
                    arm_cmd_box = gr.Textbox(label="結果", lines=2)
                    vla_set_btn.click(fn=set_vla_task, inputs=vla_task_in, outputs=vla_task_box, api_name="set_vla_task")
                    vla_get_btn.click(fn=get_vla_task, inputs=[], outputs=vla_task_box, api_name="get_vla_task")
                    arm_cmd_btn.click(fn=set_arm_cmd, inputs=arm_cmd_in, outputs=arm_cmd_box, api_name="set_arm_cmd")
                    arm_off_btn.click(fn=arm_override_off, inputs=[], outputs=arm_cmd_box, api_name="arm_override_off")

            # タブを開くタイミングでドロップダウンを現在のキーで更新
            redis_tab.select(fn=lambda: gr.update(value=REDIS_KEY_READ), outputs=redis_key_dropdown, api_name=False)
            inputbuf_tab.select(fn=lambda: gr.update(value=REDIS_KEY_READ), outputs=inputbuf_key_dropdown, api_name=False)

        # Gradioアプリを起動
        #demo.launch(server_name="0.0.0.0", server_port=7860)
        demo.launch(mcp_server=True)
        
    except Exception as e:
        print(f"[Error] Failed to start application: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit_code = main()
    exit(exit_code)
