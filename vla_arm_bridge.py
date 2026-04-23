"""
vla_arm_bridge.py - SmolVLA 右腕制御ブリッジ

映像 : Redis(meridis_frame_pub) から FPV フレームを取得
言語 : mcp-meridis get_vla_task() でタスク取得
腕制御: SmolVLA 推論 → set_arm_cmd() → mcp-meridis arm_override

使い方:
  python vla_arm_bridge.py [--redis redis.json] [--mcp http://127.0.0.1:7860]
  python vla_arm_bridge.py --mock   # SmolVLA なしで動作確認
"""
import argparse
import base64
import json
import os
import sys
import time
from io import BytesIO

try:
    import numpy as np
except ImportError:
    print("[ERROR] numpy が必要です: pip install numpy")
    sys.exit(1)

try:
    import redis as redis_lib
except ImportError:
    print("[ERROR] redis が必要です: pip install redis")
    sys.exit(1)

try:
    from gradio_client import Client as GradioClient
except ImportError:
    print("[ERROR] gradio_client が必要です: pip install gradio_client")
    sys.exit(1)

try:
    import torch
    from PIL import Image
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False

# デフォルト設定
REDIS_HOST   = "127.0.0.1"
REDIS_PORT   = 6379
FPV_KEY      = "meridis_frame_pub"
SIM_KEY      = "meridis_sim_pub"
MCP_URL      = "http://127.0.0.1:7860"
CHUNK_SIZE   = 10
STEP_INTERVAL = 0.01   # アクション送信間隔 [s] (100Hz)
POLL_INTERVAL = 0.1    # タスク未設定時のポーリング間隔 [s]

# 右腕 VAL インデックス（状態読取用）
R_ARM_VAL = [53, 55, 57, 59]   # 肩P, 肩R, 肘Y, 肘P
R_HAND_IDX = [74, 75, 76]      # 右手先 X, Y, Z [m]


def load_redis_config(json_file: str):
    global REDIS_HOST, REDIS_PORT
    if not os.path.exists(json_file):
        print(f"[WARN] 設定ファイル '{json_file}' が見つかりません。デフォルト値を使用します。")
        return
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
        if 'redis' in config:
            REDIS_HOST = config['redis'].get('host', REDIS_HOST)
            REDIS_PORT = config['redis'].get('port', REDIS_PORT)
        print(f"[INFO] Redis 設定を読み込みました: {REDIS_HOST}:{REDIS_PORT}")
    except Exception as e:
        print(f"[WARN] 設定ファイルの読み込みに失敗しました: {e}")


def get_fpv_frame(r, key: str):
    """Redis から FPV フレームを取得し [3,H,W] float32 tensor を返す。
    データ形式: JSON {"count": int, "frame": "<base64 JPEG>"} または生 base64 文字列。
    """
    raw = r.get(key)
    if raw is None:
        return None
    try:
        payload = json.loads(raw)
        frame_b64 = payload.get("frame", raw)
    except (json.JSONDecodeError, TypeError):
        frame_b64 = raw
    try:
        buf = base64.b64decode(frame_b64)
        img = Image.open(BytesIO(buf)).convert("RGB").resize((320, 240))
        arr = np.array(img, dtype=np.float32) / 255.0
        return torch.from_numpy(arr).permute(2, 0, 1)  # [3,H,W]
    except Exception as e:
        print(f"[WARN] FPV フレームのデコードに失敗しました: {e}")
        return None


def get_arm_state(r, key: str):
    """Redis から右腕状態 (4DOF) + 手先位置 (XYZ) を取得して [7] float32 tensor を返す。"""
    try:
        d = r.hgetall(key)
        arm  = [float(d.get(str(i), '0')) for i in R_ARM_VAL]
        hand = [float(d.get(str(i), '0')) for i in R_HAND_IDX]
        return torch.tensor(arm + hand, dtype=torch.float32)
    except Exception as e:
        print(f"[WARN] 腕状態の取得に失敗しました: {e}")
        return torch.zeros(7, dtype=torch.float32)


def main():
    parser = argparse.ArgumentParser(description="SmolVLA 右腕制御ブリッジ")
    parser.add_argument('--redis',   type=str, default='redis.json',
                        help='Redis 設定 JSON ファイル (default: redis.json)')
    parser.add_argument('--mcp',     type=str, default=MCP_URL,
                        help=f'mcp-meridis の Gradio URL (default: {MCP_URL})')
    parser.add_argument('--fpv-key', type=str, default=FPV_KEY,
                        help=f'FPV フレームの Redis キー (default: {FPV_KEY})')
    parser.add_argument('--sim-key', type=str, default=SIM_KEY,
                        help=f'ロボット状態の Redis キー (default: {SIM_KEY})')
    parser.add_argument('--chunk',   type=int, default=CHUNK_SIZE,
                        help=f'1 推論あたりの送信ステップ数 (default: {CHUNK_SIZE})')
    parser.add_argument('--mock',    action='store_true',
                        help='SmolVLA を使わずダミー値で動作確認する')
    args = parser.parse_args()

    load_redis_config(args.redis)

    # Redis 接続
    try:
        r = redis_lib.Redis(
            host=REDIS_HOST, port=REDIS_PORT,
            decode_responses=True,
            socket_connect_timeout=2.0,
            socket_timeout=2.0,
        )
        r.ping()
        print(f"[INFO] Redis に接続しました: {REDIS_HOST}:{REDIS_PORT}")
    except Exception as e:
        print(f"[ERROR] Redis に接続できません: {e}")
        sys.exit(1)

    # Gradio Client 接続
    try:
        mcp = GradioClient(args.mcp, verbose=False)
        print(f"[INFO] mcp-meridis に接続しました: {args.mcp}")
    except Exception as e:
        print(f"[ERROR] mcp-meridis に接続できません: {e}")
        sys.exit(1)

    # SmolVLA ロード
    policy = None
    if not args.mock:
        if not _TORCH_AVAILABLE:
            print("[ERROR] --mock なしで実行するには torch と Pillow が必要です。")
            sys.exit(1)
        try:
            from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
            print("[INFO] SmolVLA をロード中...")
            policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base").to("cuda").eval()
            print("[INFO] SmolVLA のロードが完了しました。")
        except Exception as e:
            print(f"[ERROR] SmolVLA のロードに失敗しました: {e}")
            sys.exit(1)
    else:
        print("[INFO] Mock モードで起動します（SmolVLA は使用しません）。")

    print(f"[INFO] VLA ブリッジ待機中。Claude Code チャットで set_vla_task() を呼んでください。")
    print(f"[INFO] FPV キー: {args.fpv_key}  状態キー: {args.sim_key}  チャンク: {args.chunk}")

    no_task_warned  = False
    no_frame_warned = False

    try:
        while True:
            loop_start = time.perf_counter()

            # タスク取得
            try:
                task = mcp.predict(api_name="/get_vla_task")
            except Exception as e:
                print(f"[WARN] get_vla_task 失敗: {e}")
                time.sleep(1.0)
                continue

            if not task:
                if not no_task_warned:
                    print("[INFO] タスク未設定。set_vla_task() でタスクを入力してください。")
                    no_task_warned = True
                time.sleep(max(0.0, POLL_INTERVAL - (time.perf_counter() - loop_start)))
                continue
            no_task_warned = False

            # FPV フレーム取得 (mock モードはスキップ)
            if not args.mock:
                frame = get_fpv_frame(r, args.fpv_key)
                if frame is None:
                    if not no_frame_warned:
                        print(f"[WARN] FPV フレームなし。merimujoco --stream が必要です (キー: {args.fpv_key})")
                        no_frame_warned = True
                    time.sleep(max(0.0, 0.05 - (time.perf_counter() - loop_start)))
                    continue
                no_frame_warned = False
            else:
                frame = None

            # 腕状態取得 (mock モードはスキップ)
            state = get_arm_state(r, args.sim_key) if not args.mock else None

            # 推論
            if args.mock:
                actions = [[0.0, 0.0, -90.0, 0.0]] * args.chunk
                print(f"[MOCK] task='{task}'  ダミーアクション送信 ({args.chunk} steps)")
            else:
                obs = {
                    "observation.images.fpv": frame.unsqueeze(0).cuda(),
                    "observation.state":      state.unsqueeze(0).cuda(),
                    "task":                   [task],
                }
                with torch.inference_mode():
                    action_tensor = policy.select_action(obs)  # [1, chunk, 4]
                actions = action_tensor[0].cpu().tolist()

            # アクション送信
            chunk_start = time.perf_counter()
            for step in range(min(args.chunk, len(actions))):
                try:
                    mcp.predict(json.dumps(actions[step]), api_name="/set_arm_cmd")
                except Exception as e:
                    print(f"[WARN] set_arm_cmd 失敗 (step {step}): {e}")
                    break
                # ステップ間タイミング制御
                target = chunk_start + STEP_INTERVAL * (step + 1)
                wait = target - time.perf_counter()
                if wait > 0:
                    time.sleep(wait)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C で終了します。")
    finally:
        try:
            mcp.predict("[]", api_name="/set_arm_cmd")
            print("[INFO] arm_override を無効化しました。")
        except Exception:
            pass
        print("[INFO] VLA ブリッジを終了しました。")


if __name__ == "__main__":
    main()
