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
from meri_walk_ctrl import WalkController, WalkParams, LinkParams, load_walk_params, load_link_params, save_walk_params, save_link_params
from meridim_info import MeridimKeyParams, get_key_index_text, get_system_info as _get_system_info

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
    return get_key_index_text()
# Gradio UI: MeridimKeyParamsのインデックス表示タブ
with gr.Blocks() as mrdkey_block:
    gr.Markdown("""### Meridim90 キーインデックス一覧\n各キーのインデックスと説明を表示します。""")
    key_btn = gr.Button("一覧取得")
    key_box = gr.Textbox(label="MeridimKeyParams", lines=30)
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
    with gr.Row():
        get_btn = gr.Button("取得")
        set_btn = gr.Button("設定")
    param_box = gr.Textbox(label="Params", lines=20)
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
buf_output = [[0.0] * MSG_SIZE for _ in range(10000)]  # 送信データのバッファ（10000個のdata配列を格納）
buf_input = [[0.0] * MSG_SIZE for _ in range(10000)]   # 受信データのバッファ（10000個のdata配列を格納）
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

REDIS_KEYS = ['meridis_sim_pub', 'meridis_mcp_pub', 'meridis_calc_pub', 'meridis_mgr_pub', 'meridis_console_pub']

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
def get_buf_input(start: str = "", count: str = "", decimal: str = ""):
    """
    受信バッファ(buf_input)のデータを範囲指定して取得
    buf_inputはロボットからの応答データ（IMUセンサー値、モーター実測値など）
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
        formatted_data = [f"{val:.{decimal_places}f}" for val in buf_input[i]]
        lines.append(f"[{i}] {formatted_data}")
    
    if not lines:
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
                        help='Redis configuration JSON file (default: redis-mgr.json)')
    return parser.parse_args()


# redisタブUI
with gr.Blocks() as redis_block:
    gr.Markdown("### Redisデータログ\n対象キーを選択して取得します。")
    with gr.Row():
        redis_key_dropdown = gr.Dropdown(choices=REDIS_KEYS, label="Key", value=REDIS_KEYS[0])
        redis_get_btn = gr.Button("取得")
    redis_log_box = gr.Textbox(label="Data", lines=20, elem_id="redis_log_box")
    redis_get_btn.click(fn=get_redis_data, inputs=redis_key_dropdown, outputs=redis_log_box).then(
        fn=None,
        js="() => { const el = document.querySelector('#redis_log_box textarea'); if(el) el.scrollTop = 0; }"
    )

# InputBufタブUI
with gr.Blocks() as inputbuf_block:
    gr.Markdown("""### Input Buffer (ロボットからの応答)\nbuf_inputはロボットからの応答データ（IMUセンサー値、モーター実測値など）を格納します。""")
    
    # CSV保存セクション
    btn_store_csv_in = gr.Button("buf_inputをCSVに保存")
    csv_store_result_in = gr.Textbox(label="保存結果", lines=3)
    btn_store_csv_in.click(fn=filesave_buf_input, inputs=[], outputs=csv_store_result_in)
    
    gr.Markdown("---")
    
    # データ取得セクション
    with gr.Row():
        start_read = gr.Textbox(label="開始位置", placeholder="0")
        count_read = gr.Textbox(label="量", placeholder="最後まで")
        decimal_read = gr.Textbox(label="小数点桁数", placeholder="4")
    btn_read = gr.Button("buf_input取得")
    buf_box_read = gr.Textbox(label="buf_input (受信)", lines=20)
    btn_read.click(fn=get_buf_input, inputs=[start_read, count_read, decimal_read], outputs=buf_box_read)

# OutputBufタブUI
with gr.Blocks() as outputbuf_block:
    gr.Markdown("""### Output Buffer (ロボットへの指令)\nbuf_outputはロボットへの指令データ（関節角度指令値、サーボコマンドなど）を格納します。""")
    
    # CSV保存セクション
    btn_store_csv_out = gr.Button("buf_outputをCSVに保存")
    csv_store_result_out = gr.Textbox(label="保存結果", lines=3)
    btn_store_csv_out.click(fn=filesave_buf_output, inputs=[], outputs=csv_store_result_out)
    
    gr.Markdown("---")
    
    # データ取得セクション
    with gr.Row():
        start_write = gr.Textbox(label="開始位置", placeholder="0")
        count_write = gr.Textbox(label="量", placeholder="最後まで")
        decimal_write = gr.Textbox(label="小数点桁数", placeholder="4")
    btn_write = gr.Button("buf_output取得")
    buf_box_write = gr.Textbox(label="buf_output (送信)", lines=20)
    btn_write.click(fn=get_buf_output, inputs=[start_write, count_write, decimal_write], outputs=buf_box_write)

# Controlタブ（Walk / Stop / Sysreset / Status を統合）
with gr.Blocks() as control_block:
    gr.Markdown("### Control")
    with gr.Row():
        duration_input = gr.Textbox(label="Duration", placeholder="歩行時間（秒）")
        walk_btn = gr.Button("Walk")
        stop_btn = gr.Button("Stop")
        reset_btn = gr.Button("Sysreset")
        status_btn = gr.Button("Status")
    result_out = gr.Textbox(label="Result / Status", lines=20)
    walk_btn.click(fn=robot_walk, inputs=duration_input, outputs=result_out)
    stop_btn.click(fn=robot_stop, inputs=[], outputs=result_out)
    reset_btn.click(fn=system_reset, inputs=[], outputs=result_out)
    status_btn.click(fn=robot_status, inputs=[], outputs=result_out)


def get_system_info():
    """システム情報（キーインデックスとパラメータ）を一括取得"""
    return _get_system_info(get_params_text())

# システム情報タブの追加
with gr.Blocks() as sysinfo_block:
    gr.Markdown("""### システム情報
このタブでは、Meridim90のキーインデックス、歩行パラメータ、リンクパラメータ、使用可能なスキルの一覧を一括で取得できます。
AIエージェントはこの情報を使ってシステムを理解します。""")
    sysinfo_btn = gr.Button("情報取得")
    sysinfo_box = gr.Textbox(label="System Info", lines=50)
    sysinfo_btn.click(fn=get_system_info, inputs=[], outputs=sysinfo_box)

# MeridimKeyParamsタブも追加
demo = gr.TabbedInterface(
    [control_block, params_block, redis_block, inputbuf_block, outputbuf_block, mrdkey_block, sysinfo_block],
    ["Control", "Params", "Redis", "InputBuf", "OutputBuf", "GetKeyIndex", "SysInfo"]
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