"""
redis_logger.py - PADボタントリガによるRedisデータロガー

Meridim90[15]（PADボタン値）が指定値と一致している間だけデータを
バッファに蓄積し、ボタン値が変化するかバッファ上限(10000行)に達したら
log/logs-YYYYMMDDHHMM.csv に保存する（buf_input.csv と同形式）。

使い方:
    python redis_logger.py --btn 1
    python redis_logger.py --btn 3 --redis redis-sim.json --interval 20
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import redis

REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
BUF_MAX    = 10000   # mcp-meridis.py と同じ上限
MSG_SIZE   = 90      # Meridim90 配列長
LOG_DIR    = "log"


def load_redis_config(json_file: str):
    host, port, key = REDIS_HOST, REDIS_PORT, "meridis"
    if not os.path.exists(json_file):
        print(f"Warning: '{json_file}' not found. Using defaults.", file=sys.stderr)
        return host, port, key
    try:
        with open(json_file, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        if "redis" in cfg:
            host = cfg["redis"].get("host", host)
            port = int(cfg["redis"].get("port", port))
        if "redis_keys" in cfg:
            key = cfg["redis_keys"].get("read", key)
    except Exception as e:
        print(f"Warning: Failed to load '{json_file}': {e}", file=sys.stderr)
    return host, port, key


def save_buffer(buf: list) -> str:
    os.makedirs(LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d%H%M")
    path = os.path.join(LOG_DIR, f"logs-{ts}.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for row in buf:
            writer.writerow(row)
    return path


def main():
    parser = argparse.ArgumentParser(
        description="PADボタントリガによるRedisデータロガー",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--btn", type=int, required=True,
                        help="録画トリガとなるPADボタン値 (Meridim90[15]の整数値)")
    parser.add_argument("--redis", type=str, default="redis.json",
                        help="Redis設定JSONファイル")
    parser.add_argument("--redis-key", type=str, default=None,
                        help="Redisキー名 (省略時はJSONのredis_keys.read値)")
    parser.add_argument("--interval", type=float, default=10.0,
                        help="ポーリング間隔 (ms)")
    args = parser.parse_args()

    host, port, key_from_json = load_redis_config(args.redis)
    redis_key    = args.redis_key if args.redis_key is not None else key_from_json
    interval_sec = args.interval / 1000.0
    trigger_btn  = args.btn

    client = redis.Redis(
        host=host, port=port, decode_responses=True,
        socket_connect_timeout=1.0, socket_timeout=1.0,
    )
    try:
        client.ping()
    except Exception as e:
        print(f"Error: Cannot connect to Redis at {host}:{port}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"redis_logger started.")
    print(f"  Redis     : {host}:{port}, key='{redis_key}'")
    print(f"  Trigger   : btn={trigger_btn}")
    print(f"  Interval  : {args.interval} ms")
    print(f"  Buf limit : {BUF_MAX} rows")
    print(f"  Save dir  : {LOG_DIR}/")
    print("Waiting for trigger... (Ctrl+C to quit)\n")

    buf         = []
    was_active  = False

    try:
        while True:
            t0 = time.perf_counter()

            # Redisからデータ取得
            try:
                raw  = client.hgetall(redis_key)
                data = [float(raw[str(i)]) if str(i) in raw else 0.0
                        for i in range(len(raw))] if raw else None
            except Exception:
                data = None

            btn_val = int(data[15]) if (data and len(data) > 15) else -1
            active  = (btn_val == trigger_btn)

            if active:
                if not was_active:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Recording started  (btn={btn_val})")
                # 90要素に揃えてバッファに追記
                row = (data[:MSG_SIZE] if len(data) >= MSG_SIZE
                       else data + [0.0] * (MSG_SIZE - len(data)))
                buf.append(row)

                if len(buf) >= BUF_MAX:
                    path = save_buffer(buf)
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Buffer full ({BUF_MAX} rows). Saved → {path}")
                    buf = []
                    was_active = False   # 次のトリガを待つ
                    continue

            else:
                if was_active and buf:
                    path = save_buffer(buf)
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Button changed (btn={btn_val}). "
                          f"Saved {len(buf)} rows → {path}")
                    buf = []

            was_active = active

            elapsed    = time.perf_counter() - t0
            sleep_time = interval_sec - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        if buf:
            path = save_buffer(buf)
            print(f"Saved remaining {len(buf)} rows → {path}")
    finally:
        client.close()
        print("redis_logger terminated.")


if __name__ == "__main__":
    main()
