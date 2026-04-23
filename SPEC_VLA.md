# 

## Context

### 実現したいこと（ユーザーストーリー）

> **merimujoco上で赤玉を表示した状態で、Claude Code のチャットで「赤玉に右手で触れて」と入力すると、
シミュレーション内のROID1右腕が赤玉に向かって動く。**
> 

これを実現するには以下の3つの要素が同時に成立する必要がある：

| 要素 | 内容 | 実装方針 |
| --- | --- | --- |
| **①映像入力** | ロボットFPVカメラで赤玉の位置を認識 | merimujoco が MuJoCo offscreen renderer で FPVフレームをRedisに配信 |
| **②言語指示** | 「赤玉に触れて」という自然言語コマンド | Claude Code → MCP ツール `set_vla_task()` → 英語タスク文字列を保持 |
| **③腕制御** | SmolVLAが出力する関節角度をロボットに反映 | `set_arm_cmd()` MCP ツール経由で mcp-meridis の write ループに arm_override 適用 |

### 前提：Claude Code + MCP 連携

本システムは **Claude Code CLI** から mcp-meridis MCP サーバに接続して操作する。

```bash
# Claude Code への MCP サーバ登録（初回のみ）
claude mcp add --transport sse mcp-meridis <http://127.0.0.1:7860/gradio_api/mcp/sse>
```

登録後、Claude Code のチャットで `set_vla_task()` 等の MCP ツールが直接使用できる。

---

## 既存エコシステム（確認済み）

### Redisキーとデータフロー

```
merimujoco (ml-test)
  --redis redis-ai.json  ← 要作成（read: meridis_ai_pub, write: meridis_sim_pub）
       │ write: 状態・手先位置・FPVフレーム
       ↓
  meridis_sim_pub          meridis_fpv_frame(新規)
       │                        │
       └────────────┬───────────┘
                    ↓ read
              mcp-meridis.py
              --redis redis-sim.json (read: meridis_sim_pub, write: meridis_ai_pub)
                    │ write: 全身関節指令（脚: IK歩行 + 腕: SmolVLA arm_override）
                    ↓
              meridis_ai_pub ←─── read: merimujoco
```

### Meridim90 右腕インデックス（mrd_info.py 確認済み）

| Index | 名前 | 説明 |
| --- | --- | --- |
| **52** | R_SHOULDER_P_CMD | 右肩ピッチ コマンド [deg] |
| **54** | R_SHOULDER_R_CMD | 右肩ロール コマンド [deg] |
| **56** | R_ELBOW_Y_CMD | 右肘ヨー コマンド [deg] |
| **58** | R_ELBOW_P_CMD | 右肘ピッチ コマンド [deg] |
| 74-76 | (ml-test流用) | 右手先 X,Y,Z [m] (meridis_sim_pub から取得) |

---

## 統合アーキテクチャ（最終版）

```
Claude Code チャット: 「赤玉に右手で触れて」
    ↓
Claude Code (MCP クライアント)
    ↓ set_vla_task("Touch the red ball with your right hand")
mcp-meridis.py  ←── vla_task 保持
    ↑                                    ↑
    │ GET /get_vla_task                   │ POST /set_arm_cmd
    │                                    │
    └──────── vla_arm_bridge.py ──────────┘
                    │
        ┌───────────┼─────────────────────────────┐
        ↓           ↓                             ↓
  Redis:          Redis:                    SmolVLA 推論 (Windows CUDA)
  meridis_fpv_    meridis_sim_pub           action_chunk [4DOF × 50steps]
  frame           [74-76] 右手先位置
  (JPEG base64)   [52,54,56,58] 右腕状態
        │           │                             │
        └───────────┴──────── obs ───────────────→┘
                                                  │
                               set_arm_cmd([sp,sr,ey,ep])
                                                  ↓
                                        mcp-meridis writeループ
                                        arm_override 適用
                                                  ↓
                                        meridis_ai_pub
                                        （脚: IK歩行 + 腕: SmolVLA）
                                                  ↓
                                        merimujoco / 実機ロボット
```

---

## 実装フェーズ

### Phase 1: merimujoco に FPV配信 + redis-ai.json を追加

**リポジトリ**: `merimujoco` (ml-testブランチ)

### 1-a: `redis-ai.json` 新規作成（merimujoco が ai_pub を読む設定）

```json
{
  "redis": {"host": "127.0.0.1", "port": 6379},
  "redis_keys": {"read": "meridis_ai_pub", "write": "meridis_sim_pub"}
}
```

### 1-b: `merimujoco.py` にオフスクリーンFPVレンダリングを追加 ✅ 実装済み

起動オプション `--stream` を追加し、`head_fpv` カメラでオフスクリーンレンダリングした JPEG フレームを
Redis キー `meridis_frame_pub` に Base64 エンコードして配信する。

```bash
python merimujoco.py --redis redis-ai.json --stream
```

**実装のポイント（WGLコンテキスト競合の回避）**

MuJoCo の `mujoco.Renderer` と `launch_passive` ビューアは同一 OpenGL コンテキストを共有するため、
バックグラウンドスレッドから `Renderer.update_scene/render()` を呼ぶと
`WGL: Failed to make context current: 要求されたリソースは使用中です` エラーが発生する。

→ **FPVレンダリング処理をメインスレッド（ビューアループ内）に配置**することで解決。

```python
# 初期化部分（モデルロード後・メインスレッド）
import base64
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

FPV_RENDERER = None
FPV_INTERVAL = 10        # 10ステップごと（100ms = 10fps）
FPV_REDIS_KEY = "meridis_frame_pub"

if args.stream:
    # head_fpv カメラ存在確認・Renderer 生成
    FPV_RENDERER = mujoco.Renderer(model, height=240, width=320)

# メインスレッドのビューアループ内（viewer.sync() の直後）
_fpv_cnt = 0
while viewer.is_running():
    with sim_lock:
        mujoco.mj_step(model, data)
        viewer.sync()

    # FPVレンダリング（メインスレッドで実行 → WGL競合なし）
    if FLG_STREAM and FPV_RENDERER is not None:
        _fpv_cnt += 1
        if _fpv_cnt >= FPV_INTERVAL:
            _fpv_cnt = 0
            with sim_lock:
                FPV_RENDERER.update_scene(data, camera="head_fpv")
            frame_rgb = FPV_RENDERER.render()              # [H,W,3] uint8
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            _, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            redis_transfer.redis_client.set(FPV_REDIS_KEY, base64.b64encode(buf).decode())

    time.sleep(model.opt.timestep)
```

受信・表示ツール `mrd_stream_viewer.py` も同リポジトリに追加済み：

```bash
python mrd_stream_viewer.py --redis redis-ai.json
```

### Phase 2: mcp-meridis に VLA連携ツールを追加

**リポジトリ**: `C:\\development\\mcp-meridis`

**変更ファイル**: `mcp-meridis.py`（約25行追加）

```python
# グローバル変数
arm_override: list[float] = [0.0, 0.0, 0.0, 0.0]
arm_override_enabled: bool = False
vla_task: str = ""

@mcp.tool()
def set_vla_task(task: str) -> str:
    """VLA言語タスクを設定する（Claude Codeチャットから呼び出す）
    例: set_vla_task("Touch the red ball with your right hand")
    """
    global vla_task
    vla_task = task
    return f"VLA task set: {task}"

@mcp.tool()
def get_vla_task() -> str:
    """現在のVLAタスク文字列を返す（vla_arm_bridgeがポーリング）"""
    return vla_task

@mcp.tool()
def set_arm_cmd(values: list[float]) -> str:
    """右腕コマンドを上書き [肩P, 肩R, 肘Y, 肘P] (deg)。空リストで無効化。"""
    global arm_override, arm_override_enabled
    arm_override_enabled = len(values) == 4
    if arm_override_enabled:
        arm_override = values
    return f"arm_cmd={'on' if arm_override_enabled else 'off'}: {arm_override}"

# 既存 write ループ内（meridis_ai_pub 書込み直前）に追記
if arm_override_enabled:
    data[52] = arm_override[0]  # R_SHOULDER_P_CMD
    data[54] = arm_override[1]  # R_SHOULDER_R_CMD
    data[56] = arm_override[2]  # R_ELBOW_Y_CMD
    data[58] = arm_override[3]  # R_ELBOW_P_CMD
```

### Phase 3: vla_arm_bridge.py 作成

**配置先**: `C:\\development\\meri-vla\\vla_arm_bridge.py`（本プロジェクト）

```python
"""SmolVLA 右腕制御ブリッジ
映像: Redis(meridis_fpv_frame) から FPVフレームを取得
言語: mcp-meridis get_vla_task() でタスク取得
腕制御: SmolVLA → set_arm_cmd() → mcp-meridis arm_override
"""
import redis, torch, requests, time, base64, numpy as np
from io import BytesIO
from PIL import Image
from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy

MCP = "<http://127.0.0.1:7860/gradio_api/mcp>"
R_ARM_IDX  = [52, 54, 56, 58]
R_HAND_IDX = [74, 75, 76]
CHUNK_SIZE = 10

r = redis.Redis("127.0.0.1", 6379)
policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base").to("cuda").eval()

def get_fpv_frame():
    raw = r.get("meridis_fpv_frame")
    if raw is None:
        return None
    img = Image.open(BytesIO(base64.b64decode(raw))).convert("RGB").resize((320, 240))
    arr = np.array(img, dtype=np.float32) / 255.0
    return torch.from_numpy(arr).permute(2, 0, 1)  # [3,H,W]

def get_task():
    return requests.get(f"{MCP}/get_vla_task").json().get("result", "")

def set_arm(cmd):
    requests.post(f"{MCP}/set_arm_cmd", json={"values": cmd})

print("VLA bridge ready. Waiting for set_vla_task() from Claude Code chat.")
while True:
    task = get_task()
    if not task:
        time.sleep(0.1); continue

    frame = get_fpv_frame()
    if frame is None:
        time.sleep(0.05); continue

    sim = r.hgetall("meridis_sim_pub")
    arm_s = [float(sim.get(str(i), b"0")) for i in R_ARM_IDX]
    hand  = [float(sim.get(str(i), b"0")) for i in R_HAND_IDX]
    state = torch.tensor(arm_s + hand, dtype=torch.float32)

    obs = {
        "observation.images.fpv": frame.unsqueeze(0).cuda(),
        "observation.state":      state.unsqueeze(0).cuda(),
        "task":                   [task],
    }
    with torch.inference_mode():
        actions = policy.select_action(obs)  # [1, 50, 4]

    for step in range(CHUNK_SIZE):
        set_arm(actions[0, step].cpu().tolist())
        time.sleep(0.01)  # 100Hz
```

### Phase 4: デモデータ収集（ファインチューニング用）

**配置先**: `C:\\development\\meri-vla\\collect_arm_demo.py`

- merimujoco(ml-test)でリーダー実機をテレオペ（赤玉タッチ）
- 記録: `meridis_fpv_frame`(画像) + 右腕状態 + 右手先位置 + 接触フラグ
- LeRobot Parquet + MP4 形式で `log/roid1_red_ball/` に保存

### Phase 5: ファインチューニング

```bash
cd C:\\development\\meri-vla
lerobot-train \\
  --policy.path=lerobot/smolvla_base \\
  --dataset.repo_id=holypong/roid1_red_ball \\
  --batch_size=32 --steps=20000 \\
  --output_dir=outputs/smolvla_roid1_arm
```

---

## 作成・変更ファイル一覧

| ファイル | 操作 | リポジトリ |
| --- | --- | --- |
| `redis-ai.json` | **新規** | merimujoco (ml-test) |
| `merimujoco.py` | **変更** ✅ | merimujoco (ml-test) — `--stream` オプション追加、FPV offscreen レンダリングをメインスレッドで実行 |
| `mrd_stream_viewer.py` | **新規** ✅ | merimujoco (ml-test) — Redis から `meridis_frame_pub` を受信してリアルタイム表示 |
| `mcp-meridis.py` | **変更** | mcp-meridis — 3 MCPツール追加 + arm_override（約25行） |
| `vla_arm_bridge.py` | **新規** | **meri-vla**（本プロジェクト） |
| `collect_arm_demo.py` | **新規** | **meri-vla**（本プロジェクト） |

---

## 起動手順（完成後）

```bash
# ① mcp-meridis を Claude Code の MCP として登録（初回のみ）
claude mcp add --transport sse mcp-meridis <http://127.0.0.1:7860/gradio_api/mcp/sse>

# ② Redis 初期化（merimujoco リポジトリ内で実行）
cd C:\\path\\to\\merimujoco
python create_meridis_keys.py

# ③ mcp-meridis 起動（シミュ接続: meridis_sim_pub 読取・meridis_ai_pub 書込）
cd C:\\development\\mcp-meridis
python mcp-meridis.py --redis redis-sim.json

# ④ merimujoco 起動（ml-testブランチ, meridis_ai_pub 読取・FPV配信あり）
cd C:\\path\\to\\merimujoco
python merimujoco.py --gethand true --view fpv --sphere 0.05,0.0,0.2 --redis redis-ai.json

# ⑤ VLA ブリッジ起動（タスク待受け）
cd C:\\development\\meri-vla
python vla_arm_bridge.py
```

**Claude Code チャットでの操作**:

```
ユーザー: 「赤玉に右手で触れて」
Claude Code → MCP: set_vla_task("Touch the red ball with your right hand")
（vla_arm_bridge が自動でSmolVLA推論を開始し右腕を制御）
```

---

## 検証方法

1. `meridis_fpv_frame` が Redis に書き込まれていることを確認:`python -c "import redis,base64; r=redis.Redis(); print(len(r.get('meridis_fpv_frame')), 'bytes')"`
2. Claude Code チャットで `set_vla_task("Touch the red ball")` を実行
3. `redis_plotter2.py --display joint --redis-key meridis_ai_pub` で右腕CMD(52,54,56,58)の変化を確認
4. merimujoco FPV ビューで右手が赤玉に向かう動きを目視確認
5. `touch_sphere: contact detected` ログで成功判定

---

## 実行環境

| コンポーネント | 環境 | 備考 |
| --- | --- | --- |
| Claude Code (MCP クライアント) | **Windows** | mcp-meridis を MCP サーバ登録 |
| mcp-meridis | Windows | Gradio SSE :7860 |
| merimujoco (ml-test) | Windows | redis-ai.json で meridis_ai_pub 読取 |
| SmolVLA (vla_arm_bridge) | **Windows (CUDA GPU)** | meri-vla プロジェクト |
| ml-mujoco (歩行AI学習) | WSL (参照のみ) | P9完了済み、今回は使用しない |

---

## 実装ステップ計画（Claude Code 提案）

### 参照ファイル

`mrd_stream_viewer.py`（`C:\development\merimujoco\`）から以下のパターンを流用する:

- `load_redis_config()` + argparse 構造
- フレームデコード（JSON `{"count":..., "frame":"<base64>"}` への対応）
- Redis 接続・ping チェック、graceful shutdown

---

### Step 1: mcp-meridis.py に Phase 2 変数・関数を追加

SPEC の「約25行」を実装する。

```python
# グローバル変数（既存変数定義の近くに追加）
arm_override: list = [0.0, 0.0, 0.0, 0.0]
arm_override_enabled: bool = False
vla_task: str = ""

# Gradio API / MCP ツールとして公開する関数
def set_vla_task(task: str) -> str: ...
def get_vla_task() -> str: ...
def set_arm_cmd(values: list) -> str: ...
```

`arm_override` の適用場所 → `_arm_send_angles()` の `transfer.set_data()` 直前に挿入
（歩行ループ・腕IK送信の両方に自動適用）。

Gradio UI タブには追加不要。`mcp_server=True` 経由で外部から呼べるようにする。

---

### Step 2: 通信方式の確定

SPEC の `requests.get/post` は Gradio MCP の SSE と噛み合わない。
代わりに **`gradio_client.Client`** を使う:

```python
from gradio_client import Client
mcp = Client("http://127.0.0.1:7860")
task = mcp.predict(api_name="/get_vla_task")
mcp.predict(cmd_list, api_name="/set_arm_cmd")
```

これにより型チェック・エラーハンドリングが整備された形で mcp-meridis と通信できる。

---

### Step 3: vla_arm_bridge.py の骨格作成（SmolVLA なしで動作確認できる形）

`mrd_stream_viewer.py` の構造を踏まえ以下の構成とする:

```
main()
├── parse_arguments()  --redis, --mcp-url, --fps, --mock
├── load_redis_config()  （mrd_stream_viewer と同パターン）
├── Redis 接続 + ping
├── Gradio Client 接続
├── SmolVLA ロード（--mock なら省略）
└── メインループ
    ├── get_vla_task() → 空なら skip
    ├── get_fpv_frame()（mrd_stream_viewer のデコードロジック流用）
    ├── Redis から右腕状態 [53,55,57,59] + 手先位置 [74,75,76] 読取
    ├── obs 構築 → policy.select_action()（mock 時はランダム値）
    └── action_chunk を CHUNK_SIZE ステップ分 set_arm_cmd() 送信
```

---

### Step 4: SmolVLA obs フォーマット調整

SPEC の `obs` を LeRobot の SmolVLA API に合わせる:

```python
obs = {
    "observation.images.fpv": frame_tensor,   # [1,3,H,W] float32 0-1
    "observation.state": state_tensor,         # [1,7] (腕4DOF + 手先XYZ3)
    "task": [task_str],
}
actions = policy.select_action(obs)  # [1, chunk, 4]
```

LeRobot の実際の API に合わせて調整が必要な箇所があるため、
Step 3 で mock 動作を確認してから統合する。

---

### Step 5: 配置先（確定）

**`C:\development\mcp-meridis\vla_arm_bridge.py`** に作成する。
Phase 2 の mcp-meridis.py 変更と同リポジトリで管理する。

---

### 実施順序まとめ

| # | 作業 | ファイル | 依存 |
|---|------|---------|------|
| 1 | グローバル変数・3関数追加 + arm_override 適用 | `mcp-meridis.py` | なし |
| 2 | Gradio API 公開確認（`/get_vla_task` 等） | `mcp-meridis.py` | Step 1 |
| 3 | 骨格作成（mock モード付き） | `vla_arm_bridge.py` | Step 2 |
| 4 | FPV フレーム受信確認（mrd_stream_viewer 流用） | `vla_arm_bridge.py` | Redis / merimujoco 起動 |
| 5 | SmolVLA 統合 | `vla_arm_bridge.py` | CUDA 環境・lerobot |

---

## 確定事項

1. **`vla_arm_bridge.py` の配置先**: `C:\development\mcp-meridis\` ✅
2. **FPV フレーム Redis キー名**: `meridis_frame_pub` ✅
3. **実施順序**: Step 1（mcp-meridis.py）→ vla_arm_bridge.py の順に実施 ✅

---

## 検証記録

### Step 1 + Step 3 mock 疎通確認 ✅ 完了（2026-04-23）

**実行コマンド**:
```bash
python vla_arm_bridge.py --redis redis-sim.json --mock
```

**確認結果**:

| チェック項目 | 結果 |
|---|---|
| Redis 接続（127.0.0.1:6379） | ✅ OK |
| mcp-meridis Gradio 接続（127.0.0.1:7860） | ✅ OK |
| `get_vla_task()` API ポーリング | ✅ OK（空タスク時は待機メッセージ1回のみ） |
| タスク設定後の検出 | ✅ OK（`'Touch the red ball'` を正しく受信） |
| `set_arm_cmd()` mock 送信（10 steps） | ✅ OK（エラーなし） |

**確認ログ（抜粋）**:
```
[INFO] Redis に接続しました: 127.0.0.1:6379
[INFO] mcp-meridis に接続しました: http://127.0.0.1:7860
[INFO] タスク未設定。set_vla_task() でタスクを入力してください。
[MOCK] task='Touch the red ball'  ダミーアクション送信 (10 steps)
```

**注意事項**:
- タスクは `set_vla_task()` の引数にテキストのみ渡す（関数呼び出し構文ごと入れない）
- `--mock` モードでは FPV フレーム取得・腕状態取得をスキップするため PIL/torch 不要

**次ステップ**: Step 4 — merimujoco `--stream` 起動後に実 FPV フレーム受信を確認
