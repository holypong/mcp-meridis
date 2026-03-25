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
import re
from walk_ctrl import WalkController, WalkParams, LinkParams, load_walk_params, load_link_params, save_walk_params, save_link_params
from meridim_keys import MeridimKeyParams, get_meridim_key_meta

# 20260103 安定版

# 定数
MSG_SIZE = 90               # Meridim配列の長さ
MSG_BUFF = MSG_SIZE * 2     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2     # Meridim配列のエラーフラグの格納場所
MSG_CKSM = MSG_SIZE - 1     # Meridim配列のチェックサムの格納場所
# Redisサーバー設定（デフォルト値、JSONファイルから読み込まれる）
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_KEY_READ = "meridis_sim_pub"
REDIS_KEY_WRITE = "meridis_mcp_pub"

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


# MeridimKeyParamsのインデックスと説明を表示する関数
def getmrdkey():
    meta = get_meridim_key_meta()
    lines = [f"{k}: {v['index']}  # {v['description']}" for k, v in meta.items()]
    return '\n'.join(lines)
# Gradio UI: MeridimKeyParamsのインデックス表示タブ
with gr.Blocks() as mrdkey_block:
    gr.Markdown("""### Meridim90 キーインデックス一覧\n各キーのインデックスと説明を表示します。""")
    key_box = gr.Textbox(label="MeridimKeyParams", lines=30)
    key_btn = gr.Button("一覧取得")
    key_btn.click(fn=getmrdkey, inputs=[], outputs=key_box)

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
    
    # パラメータをJSONファイルに保存
    save_walk_params(params, "walkparam.json")
    save_link_params(params_link, "linkparam.json")
    
    return get_params_text()
# Gradio パラメータ一括取得・一括設定UI
with gr.Blocks() as params_block:
    gr.Markdown("""### パラメータ一括取得・一括設定
1. [取得]ボタンで現在値をテキストボックスに表示
2. 編集後、[設定]ボタンで一括反映
""")
    param_box = gr.Textbox(label="Params", lines=20)
    with gr.Row():
        get_btn = gr.Button("取得")
        set_btn = gr.Button("設定")
    get_btn.click(fn=get_params_text, inputs=[], outputs=param_box)
    set_btn.click(fn=set_params_text, inputs=param_box, outputs=param_box)


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
w_sts = 0

# 歩行ループ
t = 0
foot_ref_pitch = np.radians(0.0)  # 基準となる足首ピッチ角


# float型のdataの初期化
data = [0.0] * MSG_SIZE

# バッファ変数
output_buf = [[0.0] * MSG_SIZE for _ in range(10000)]  # 送信データのバッファ（10000個のdata配列を格納）
input_buf = [[0.0] * MSG_SIZE for _ in range(10000)]   # 受信データのバッファ（10000個のdata配列を格納）
buf_index = 0  # インクリメンタルカウンタ


# WalkParamsとLinkParamsはwalk_ctrlからインポート
# JSONファイルから読み込み（なければデフォルト値を使用）
params = load_walk_params("walkparam.json")
print(f"Loaded WalkParams: cycle_duration={params.cycle_duration}")

# Redis設定とクライアント初期化は main() 関数で行う


# リンク長の定義と脚全体の長さを構造体にまとめる
# LinkParamsはwalk_ctrlからインポート
# JSONファイルから読み込み（なければデフォルト値を使用）
params_link = load_link_params("linkparam.json")
print(f"Loaded LinkParams: THIGH_LENGTH={params_link.THIGH_LENGTH}, SHANK_LENGTH={params_link.SHANK_LENGTH}")


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
    global MOT_STS, stop_background, data, t, output_buf, buf_index, w_sts
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
            output_buf = walk_controller.output_buf
            input_buf = walk_controller.input_buf
            buf_index = walk_controller.buf_index
            
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
    global MOT_STS, data, output_buf, input_buf, buf_index

    if(command == "walk"):
        walk_controller.start_walk()
        MOT_STS = WALK
        data = walk_controller.data
        output_buf = walk_controller.output_buf
        input_buf = walk_controller.input_buf
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

    elif(command == "reset"):
        MOT_STS = IDLE
        stop_background_thread()

        walk_controller.reset_pose()
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
    """ロボット停止"""
    return meridian_command("stop", "", "")


# Resetタブ用のリセット関数
def system_reset():
    """Resetタブからのリセット操作。1回だけdata[0]=5556で送信"""
    global data
    data[0] = 5556
    transfer.set_data(REDIS_KEY_WRITE, data)
    time.sleep(0.01)  # 少し待ってから元に戻す
    data[0] = 0.0  # 送信後は元に戻す    
    transfer.set_data(REDIS_KEY_WRITE, data)
    return "リセット信号（data[0]=5556）を1回送信しました。"

def robot_status():
    """ロボット状態確認"""
    global MOT_STS, t, w_sts
    status_map = {IDLE: "停止中", WALK: "歩行中"}
    current_status = status_map.get(MOT_STS, "不明")
    # 列挙形式で複数行出力
    lines = [
        f"1. 状態: {current_status}",
        f"2. 時間: {t:.2f}秒",
        f"3. 歩行段階: {w_sts}"
    ]
    # 20行に満たない場合は空行で埋める
    while len(lines) < 20:
        lines.append("")
    return "\n".join(lines)

# Redisデータ取得関数

# meridis データ取得（1行1要素で表示）
def get_redis_meridis():
    try:
        # receiverが保持しているredis_keyを使用
        key = receiver.redis_key if receiver else REDIS_KEY_READ
        d = receiver.redis_client.hgetall(key)
        if not d:
            return f"(no data for key: {key})"
        arr = [float(d[str(i)]) if str(i) in d else None for i in range(len(d))]
        lines = [f"Key: {key}", "---"] + [f"[{i}] {v}" for i, v in enumerate(arr)]
        return "\n".join(lines)
    except Exception as e:
        return f"error: {e}\nKey: {receiver.redis_key if receiver else 'N/A'}"

# meridis_mcp_pub データ取得（1行1要素で表示）
def get_redis_meridis_mcp_pub():
    try:
        # transferが保持しているredis_keyを使用
        key = transfer.redis_key if transfer else REDIS_KEY_WRITE
        d = transfer.redis_client.hgetall(key)
        if not d:
            return f"(no data for key: {key})"
        arr = [float(d[str(i)]) if str(i) in d else None for i in range(len(d))]
        lines = [f"Key: {key}", "---"] + [f"[{i}] {v}" for i, v in enumerate(arr)]
        return "\n".join(lines)
    except Exception as e:
        return f"error: {e}\nKey: {transfer.redis_key if transfer else 'N/A'}"


# output_buf/input_bufデータ取得関数

# output_bufデータ取得（1行1要素で表示）
def get_output_buf(start: str = "", count: str = "", decimal: str = ""):
    """
    送信バッファ(output_buf)のデータを範囲指定して取得
    output_bufはロボットへの指令データ（関節角度指令値、サーボコマンドなど）
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
        formatted_data = [f"{val:.{decimal_places}f}" for val in output_buf[i]]
        lines.append(f"[{i}] {formatted_data}")
    
    if not lines:
        return "(no data in specified range)"
    return "\n".join(lines)

# input_bufデータ取得（1行1要素で表示）
def get_input_buf(start: str = "", count: str = "", decimal: str = ""):
    """
    受信バッファ(input_buf)のデータを範囲指定して取得
    input_bufはロボットからの応答データ（IMUセンサー値、モーター実測値など）
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
        formatted_data = [f"{val:.{decimal_places}f}" for val in input_buf[i]]
        lines.append(f"[{i}] {formatted_data}")
    
    if not lines:
        return "(no data in specified range)"
    return "\n".join(lines)


# CSV保存・読み込み関数

def filesave_input_buf():
    """
    input_buf(ロボットからの応答データ)をCSVファイルに保存（固定ファイル名で上書き）
    Returns:
        保存結果のメッセージ
    """
    if buf_index == 0:
        return "エラー: 保存するデータがありません"
    
    try:
        # 固定ファイル名
        filename = "input_buf.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 生データのみ書き込み（ヘッダーなし、インデックスなし）
            for i in range(buf_index):
                writer.writerow(input_buf[i])
        
        abs_path = os.path.abspath(filename)
        return f"保存完了: {filename}\nパス: {abs_path}\nデータ数: {buf_index}"
    except Exception as e:
        return f"エラー: {str(e)}"

def filesave_output_buf():
    """
    output_buf(ロボットへの指令データ)をCSVファイルに保存（固定ファイル名で上書き）
    Returns:
        保存結果のメッセージ
    """
    if buf_index == 0:
        return "エラー: 保存するデータがありません"
    
    try:
        # 固定ファイル名
        filename = "output_buf.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 生データのみ書き込み（ヘッダーなし、インデックスなし）
            for i in range(buf_index):
                writer.writerow(output_buf[i])
        
        abs_path = os.path.abspath(filename)
        return f"保存完了: {filename}\nパス: {abs_path}\nデータ数: {buf_index}"
    except Exception as e:
        return f"エラー: {str(e)}"

def filepathget_input_buf():
    """
    input_buf(ロボットからの応答データ) CSVファイルのパスを返す（AIエージェント用）
    AIエージェントはこのパスを使ってファイルを直接読み取ることができます
    Returns:
        絶対パスまたはエラーメッセージ
    """
    try:
        # 固定ファイル名のinput_buf.csvを使用
        filename = "input_buf.csv"
        
        if os.path.exists(filename):
            return os.path.abspath(filename)
        else:
            return "エラー: input_buf.csvが見つかりません。先にCSV保存を実行してください。"
    except Exception as e:
        return f"エラー: {str(e)}"

def filepathget_output_buf():
    """
    output_buf(ロボットへの指令データ) CSVファイルのパスを返す（AIエージェント用）
    AIエージェントはこのパスを使ってファイルを直接読み取ることができます
    Returns:
        絶対パスまたはエラーメッセージ
    """
    try:
        # 固定ファイル名のoutput_buf.csvを使用
        filename = "output_buf.csv"
        
        if os.path.exists(filename):
            return os.path.abspath(filename)
        else:
            return "エラー: output_buf.csvが見つかりません。先にCSV保存を実行してください。"
    except Exception as e:
        return f"エラー: {str(e)}"

def parse_arguments():
    """コマンドライン引数を解析する"""
    parser = argparse.ArgumentParser(description='MCP Meridis - Gradio Web Interface for Robot Control')
    parser.add_argument('--redis',
                        default='redis.json',
                        help='Redis configuration JSON file (default: redis-mgr.json)')
    return parser.parse_args()


# redisタブUI（表示エリア分離）
with gr.Blocks() as redis_block:
    gr.Markdown("""### Redisデータログ\nmeridis・meridis_mcp_pubの現在値をそれぞれ取得して表示します。""")
    with gr.Row():
        with gr.Column():
            btn_a = gr.Button(f"{REDIS_KEY_READ}取得")    
            log_box_a = gr.Textbox(label=REDIS_KEY_READ, lines=20)
        with gr.Column():
            btn_b = gr.Button(f"{REDIS_KEY_WRITE}取得")
            log_box_b = gr.Textbox(label=REDIS_KEY_WRITE, lines=20)
    btn_a.click(fn=get_redis_meridis, inputs=[], outputs=log_box_a)
    btn_b.click(fn=get_redis_meridis_mcp_pub, inputs=[], outputs=log_box_b)

# InputBufタブUI
with gr.Blocks() as inputbuf_block:
    gr.Markdown("""### Input Buffer (ロボットからの応答)\ninput_bufはロボットからの応答データ（IMUセンサー値、モーター実測値など）を格納します。""")
    
    # CSV保存・パス取得セクション
    with gr.Row():
        with gr.Column():
            btn_store_csv_in = gr.Button("input_bufをCSVに保存")
            csv_store_result_in = gr.Textbox(label="保存結果", lines=3)
        with gr.Column():
            btn_path_csv_in = gr.Button("CSVファイルパス取得")
            csv_path_result_in = gr.Textbox(label="ファイルパス", lines=3)
    
    btn_store_csv_in.click(fn=filesave_input_buf, inputs=[], outputs=csv_store_result_in)
    btn_path_csv_in.click(fn=filepathget_input_buf, inputs=[], outputs=csv_path_result_in)
    
    gr.Markdown("---")
    
    # データ取得セクション
    with gr.Row():
        start_read = gr.Textbox(label="開始位置", placeholder="0")
        count_read = gr.Textbox(label="量", placeholder="最後まで")
        decimal_read = gr.Textbox(label="小数点桁数", placeholder="4")
    btn_read = gr.Button("input_buf取得")
    buf_box_read = gr.Textbox(label="input_buf (受信)", lines=20)
    btn_read.click(fn=get_input_buf, inputs=[start_read, count_read, decimal_read], outputs=buf_box_read)

# OutputBufタブUI
with gr.Blocks() as outputbuf_block:
    gr.Markdown("""### Output Buffer (ロボットへの指令)\noutput_bufはロボットへの指令データ（関節角度指令値、サーボコマンドなど）を格納します。""")
    
    # CSV保存・パス取得セクション
    with gr.Row():
        with gr.Column():
            btn_store_csv_out = gr.Button("output_bufをCSVに保存")
            csv_store_result_out = gr.Textbox(label="保存結果", lines=3)
        with gr.Column():
            btn_path_csv_out = gr.Button("CSVファイルパス取得")
            csv_path_result_out = gr.Textbox(label="ファイルパス", lines=3)
    
    btn_store_csv_out.click(fn=filesave_output_buf, inputs=[], outputs=csv_store_result_out)
    btn_path_csv_out.click(fn=filepathget_output_buf, inputs=[], outputs=csv_path_result_out)
    
    gr.Markdown("---")
    
    # データ取得セクション
    with gr.Row():
        start_write = gr.Textbox(label="開始位置", placeholder="0")
        count_write = gr.Textbox(label="量", placeholder="最後まで")
        decimal_write = gr.Textbox(label="小数点桁数", placeholder="4")
    btn_write = gr.Button("output_buf取得")
    buf_box_write = gr.Textbox(label="output_buf (送信)", lines=20)
    btn_write.click(fn=get_output_buf, inputs=[start_write, count_write, decimal_write], outputs=buf_box_write)

# Walkタブ（Blocks形式に統一）
with gr.Blocks() as walk_demo:
    gr.Markdown("""### Walk
歩行時間を入力してGenerateボタンを押すと歩行を開始します。""")
    with gr.Row():
        duration_input = gr.Textbox(label="Duration", placeholder="歩行時間（秒）")
        walk_btn = gr.Button("Generate")
        walk_out = gr.Textbox(label="Result", lines=2)
    walk_btn.click(fn=robot_walk, inputs=duration_input, outputs=walk_out)

# Stopタブ（Blocks形式に統一）
with gr.Blocks() as stop_demo:
    gr.Markdown("""### Stop
Generateボタンを押すとロボットを停止します。""")
    with gr.Row():
        stop_btn = gr.Button("Generate")
        stop_out = gr.Textbox(label="Result", lines=2)
    stop_btn.click(fn=robot_stop, inputs=[], outputs=stop_out)

# Resetタブ（フォーム形式、Generateボタンで実行）
with gr.Blocks() as reset_block:
    gr.Markdown("""### Reset
Generateボタンを押すと、リセット信号（data[0]=5556）を1回だけ送信します。""")
    with gr.Row():
        reset_btn = gr.Button("Generate")
        reset_out = gr.Textbox(label="Result", lines=2)    
    reset_btn.click(fn=system_reset, inputs=[], outputs=reset_out)

# Statusタブ（Blocks形式に統一）
with gr.Blocks() as status_demo:
    gr.Markdown("""### Status
Generateボタンを押すとロボットの状態を確認できます。""")
    with gr.Row():
        status_btn = gr.Button("Generate")
        status_out = gr.Textbox(label="Status", lines=20)
    status_btn.click(fn=robot_status, inputs=[], outputs=status_out)


# システム情報を返す専用の関数を追加
def get_system_info():
    """システム情報（キーインデックスとパラメータ）を一括取得"""
    info_parts = []
    
    # 1. Meridim90キーインデックス情報
    info_parts.append("=== Meridim90 キーインデックス一覧 ===")
    info_parts.append(getmrdkey())
    info_parts.append("")
    
    # 2. 歩行パラメータとリンクパラメータ
    info_parts.append("=== 現在の歩行パラメータとリンクパラメータ ===")
    info_parts.append(get_params_text())
    info_parts.append("")
    
    # 3. 使用方法の説明
    info_parts.append("=== 使用可能なツール ===")
    info_parts.append("- getmrdkey: Meridim90のキーインデックス一覧を取得")
    info_parts.append("- get_params_text: 現在のパラメータを取得")
    info_parts.append("- set_params_text: パラメータを一括設定")
    info_parts.append("- robot_walk: ロボットを歩行させる（duration指定可能）")
    info_parts.append("- robot_stop: ロボットを停止")
    info_parts.append("- robot_status: ロボットの状態を確認")
    info_parts.append("- system_reset: システムリセット")
    info_parts.append("- get_input_buf: ロボットからの応答データを取得")
    info_parts.append("- get_output_buf: ロボットへの指令データを取得")
    info_parts.append("- filesave_input_buf: input_buf(ロボットからの応答)をCSVファイルに保存")
    info_parts.append("- filesave_output_buf: output_buf(ロボットへの指令)をCSVファイルに保存")
    info_parts.append("- filepathget_input_buf: input_buf CSVファイルの絶対パスを取得（このパスでread_fileスキルを使ってファイルを直接読み取り可能）")
    info_parts.append("- filepathget_output_buf: output_buf CSVファイルの絶対パスを取得（このパスでread_fileスキルを使ってファイルを直接読み取り可能）")
    info_parts.append("- get_system_info: システム情報を一括取得（このスキル）")
    info_parts.append("")
    
    return "\n".join(info_parts)

# システム情報タブの追加
with gr.Blocks() as sysinfo_block:
    gr.Markdown("""### システム情報
このタブでは、Meridim90のキーインデックス、歩行パラメータ、リンクパラメータ、使用可能なスキルの一覧を一括で取得できます。
AIエージェントはこの情報を使ってシステムを理解します。""")
    sysinfo_box = gr.Textbox(label="System Info", lines=50)
    sysinfo_btn = gr.Button("情報取得")
    sysinfo_btn.click(fn=get_system_info, inputs=[], outputs=sysinfo_box)

# MeridimKeyParamsタブも追加
demo = gr.TabbedInterface(
    [walk_demo, stop_demo, reset_block, status_demo, params_block, redis_block, inputbuf_block, outputbuf_block, mrdkey_block, sysinfo_block],
    ["Walk", "Stop", "Sysreset", "Status", "Params", "Redis", "InputBuf", "OutputBuf", "GetKeyIndex", "SysInfo"]
)

def main():
    """メイン関数 - コマンドライン引数を処理してGradioアプリを起動"""
    global receiver, transfer, walk_controller
    
    try:
        # コマンドライン引数を解析
        args = parse_arguments()
        
        # Redis設定をJSONファイルから読み込み
        load_redis_config(args.redis)
        
        # Redisクライアントを初期化
        receiver = redis_receiver.RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)
        transfer = redis_transfer.RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)
        
        # WalkControllerインスタンスを作成
        walk_controller = WalkController(params=params, params_link=params_link, msg_size=MSG_SIZE)
        start_background_thread()

        print(f"[Info] Starting Gradio web interface...")
        print(f"[Info] Redis config loaded from: {args.redis}")
        
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