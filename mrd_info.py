"""
Meridim90 キーインデックス定義とシステム情報
Meridim90配列の各インデックス・説明の定義、およびシステム情報の生成を担う
"""
from dataclasses import dataclass, field, fields


def key_field(index, description):
    return field(default=index, metadata={"description": description})


@dataclass(frozen=True)
class MeridimKeyParams:
    MRD_MASTER: int = key_field(0,  "マスターコマンド")
    MRD_SEQ: int = key_field(1,  "シーケンス番号")
    MRD_ACC_X: int = key_field(2,  "加速度センサX値")
    MRD_ACC_Y: int = key_field(3,  "加速度センサY値")
    MRD_ACC_Z: int = key_field(4,  "加速度センサZ値")
    MRD_GYRO_X: int = key_field(5,  "ジャイロセンサX値")
    MRD_GYRO_Y: int = key_field(6,  "ジャイロセンサY値")
    MRD_GYRO_Z: int = key_field(7,  "ジャイロセンサZ値")
    MRD_MAG_X: int = key_field(8,  "磁気コンパスX値")
    MRD_MAG_Y: int = key_field(9,  "磁気コンパスY値")
    MRD_MAG_Z: int = key_field(10, "磁気コンパスZ値")
    MRD_TEMP: int = key_field(11, "温度センサ値")
    MRD_DIR_ROLL: int = key_field(12, "DMP推定ロール方向値")
    MRD_DIR_PITCH: int = key_field(13, "DMP推定ピッチ方向値")
    MRD_DIR_YAW: int = key_field(14, "DMP推定ヨー方向値")
    MRD_PAD_BUTTONS: int = key_field(15, "リモコンの基本ボタン値")
    MRD_PAD_STICK_L: int = key_field(16, "リモコンの左スティックアナログ値")
    MRD_PAD_STICK_R: int = key_field(17, "リモコンの右スティックアナログ値")
    MRD_PAD_L2R2VAL: int = key_field(18, "リモコンのL2R2ボタンアナログ値")
    MRD_MOTION_FRAMES: int = key_field(19, "モーション設定のフレーム数")
    C_HEAD_Y_CMD: int = key_field(20, "頭ヨーのコマンド")
    C_HEAD_Y_VAL: int = key_field(21, "頭ヨーの値")
    L_SHOULDER_P_CMD: int = key_field(22, "左肩ピッチのコマンド")
    L_SHOULDER_P_VAL: int = key_field(23, "左肩ピッチの値")
    L_SHOULDER_R_CMD: int = key_field(24, "左肩ロールのコマンド")
    L_SHOULDER_R_VAL: int = key_field(25, "左肩ロールの値")
    L_ELBOW_Y_CMD: int = key_field(26, "左肘ヨーのコマンド")
    L_ELBOW_Y_VAL: int = key_field(27, "左肘ヨーの値")
    L_ELBOW_P_CMD: int = key_field(28, "左肘ピッチのコマンド")
    L_ELBOW_P_VAL: int = key_field(29, "左肘ピッチの値")
    L_HIPJOINT_Y_CMD: int = key_field(30, "左股ヨーのコマンド")
    L_HIPJOINT_Y_VAL: int = key_field(31, "左股ヨーの値")
    L_HIPJOINT_R_CMD: int = key_field(32, "左股ロールのコマンド")
    L_HIPJOINT_R_VAL: int = key_field(33, "左股ロールの値")
    L_HIPJOINT_P_CMD: int = key_field(34, "左股ピッチのコマンド")
    L_HIPJOINT_P_VAL: int = key_field(35, "左股ピッチの値")
    L_KNEE_P_CMD: int = key_field(36, "左膝ピッチのコマンド")
    L_KNEE_P_VAL: int = key_field(37, "左膝ピッチの値")
    L_ANKLE_P_CMD: int = key_field(38, "左足首ピッチのコマンド")
    L_ANKLE_P_VAL: int = key_field(39, "左足首ピッチの値")
    L_ANKLE_R_CMD: int = key_field(40, "左足首ロールのコマンド")
    L_ANKLE_R_VAL: int = key_field(41, "左足首ロールの値")
    L_SERVO_IX11_CMD: int = key_field(42, "追加サーボ用のコマンド")
    L_SERVO_IX11_VAL: int = key_field(43, "追加サーボ用の値")
    L_SERVO_IX12_CMD: int = key_field(44, "追加サーボ用のコマンド")
    L_SERVO_IX12_VAL: int = key_field(45, "追加サーボ用の値")
    L_SERVO_IX13_CMD: int = key_field(46, "追加サーボ用のコマンド")
    L_FOOT_X: int = key_field(47, "左足X位置（追加サーボ用の値を流用・mcp-meridis/redis_plotterで使用）")
    L_FOOT_Y: int = key_field(48, "左足Y位置（追加サーボ用のコマンドを流用・mcp-meridis/redis_plotterで使用）")
    L_FOOT_Z: int = key_field(49, "左足Z位置（追加サーボ用の値を流用・mcp-meridis/redis_plotterで使用）")
    C_WAIST_Y_CMD: int = key_field(50, "腰ヨーのコマンド")
    C_WAIST_Y_VAL: int = key_field(51, "腰ヨーの値")
    R_SHOULDER_P_CMD: int = key_field(52, "右肩ピッチのコマンド")
    R_SHOULDER_P_VAL: int = key_field(53, "右肩ピッチの値")
    R_SHOULDER_R_CMD: int = key_field(54, "右肩ロールのコマンド")
    R_SHOULDER_R_VAL: int = key_field(55, "右肩ロールの値")
    R_ELBOW_Y_CMD: int = key_field(56, "右肘ヨーのコマンド")
    R_ELBOW_Y_VAL: int = key_field(57, "右肘ヨーの値")
    R_ELBOW_P_CMD: int = key_field(58, "右肘ピッチのコマンド")
    R_ELBOW_P_VAL: int = key_field(59, "右肘ピッチの値")
    R_HIPJOINT_Y_CMD: int = key_field(60, "右股ヨーのコマンド")
    R_HIPJOINT_Y_VAL: int = key_field(61, "右股ヨーの値")
    R_HIPJOINT_R_CMD: int = key_field(62, "右股ロールのコマンド")
    R_HIPJOINT_R_VAL: int = key_field(63, "右股ロールの値")
    R_HIPJOINT_P_CMD: int = key_field(64, "右股ピッチのコマンド")
    R_HIPJOINT_P_VAL: int = key_field(65, "右股ピッチの値")
    R_KNEE_P_CMD: int = key_field(66, "右膝ピッチのコマンド")
    R_KNEE_P_VAL: int = key_field(67, "右膝ピッチの値")
    R_ANKLE_P_CMD: int = key_field(68, "右足首ピッチのコマンド")
    R_ANKLE_P_VAL: int = key_field(69, "右足首ピッチの値")
    R_ANKLE_R_CMD: int = key_field(70, "右足首ロールのコマンド")
    R_ANKLE_R_VAL: int = key_field(71, "右足首ロールの値")
    R_SERVO_IX11_CMD: int = key_field(72, "追加テスト用のコマンド")
    R_SERVO_IX11_VAL: int = key_field(73, "追加テスト用の値")
    R_SERVO_IX12_CMD: int = key_field(74, "追加テスト用のコマンド")
    R_SERVO_IX12_VAL: int = key_field(75, "追加テスト用の値")
    R_SERVO_IX13_CMD: int = key_field(76, "追加テスト用のコマンド")
    R_FOOT_X: int = key_field(77, "右足X位置（追加テスト用の値を流用・mcp-meridis/redis_plotterで使用）")
    R_FOOT_Y: int = key_field(78, "右足Y位置（追加テスト用のコマンドを流用・mcp-meridis/redis_plotterで使用）")
    R_FOOT_Z: int = key_field(79, "右足Z位置（追加テスト用の値を流用・mcp-meridis/redis_plotterで使用）")
    MRD_USERDATA_80: int = key_field(80, "ユーザー定義用")
    MRD_USERDATA_81: int = key_field(81, "ユーザー定義用")
    MRD_USERDATA_82: int = key_field(82, "ユーザー定義用")
    MRD_USERDATA_83: int = key_field(83, "ユーザー定義用")
    MRD_USERDATA_84: int = key_field(84, "ユーザー定義用")
    MRD_USERDATA_85: int = key_field(85, "ユーザー定義用")
    MRD_USERDATA_86: int = key_field(86, "ユーザー定義用")
    MRD_USERDATA_87: int = key_field(87, "ユーザー定義用")
    MRD_ERR: int = key_field(88, "エラーコード")
    MRD_CKSM: int = key_field(89, "チェックサム")


def get_meridim_key_meta():
    """
    MeridimKeyParamsの各キーのインデックスと説明を辞書で返す
    { 'MRD_MASTER': { 'index': 0, 'description': 'マスターコマンド' }, ... }
    """
    return {
        f.name: {
            "index": getattr(MeridimKeyParams(), f.name),
            "description": f.metadata.get("description", "")
        }
        for f in fields(MeridimKeyParams)
    }


def get_key_index_text():
    """Meridim90キーインデックス一覧をテキストで返す"""
    meta = get_meridim_key_meta()
    lines = [f"{k}: {v['index']}  # {v['description']}" for k, v in meta.items()]

    # デバッグプリント
    #for line in lines:
    #     print(line)

    return '\n'.join(lines)


def get_system_info(params_text: str, startup_options_text: str = "") -> str:
    """システム情報（キーインデックス・パラメータ・ツール一覧）を一括取得"""
    return "\n".join([
        "=== Meridim90 キーインデックス一覧 ===",
        get_key_index_text(),
        "",
        "=== 起動時のオプション ===",
        startup_options_text,
        "",
        "=== 現在の歩行パラメータとリンクパラメータ ===",
        params_text,
        "",
        "=== 使用可能なツール ===",
        "- getmrdkey: Meridim90のキーインデックス一覧を取得",
        "- get_params_text: 現在のパラメータを取得",
        "- get_startup_options: 起動時のオプションと指定値を取得",
        "- set_params_text: パラメータを一括設定",
        "- robot_walk: ロボットを歩行させる（duration指定可能）",
        "- robot_stop: ロボットを停止",
        "- robot_status: ロボットの状態を確認",
        "- system_reset: システムリセット",
        "- get_buf_input: ロボットからの応答データを取得",
        "- get_buf_output: ロボットへの指令データを取得",
        "- filesave_buf_input: buf_input(ロボットからの応答)をCSVファイルに保存",
        "- filesave_buf_output: buf_output(ロボットへの指令)をCSVファイルに保存",
        "- filepathget_buf_input: buf_input CSVファイルの絶対パスを取得",
        "- filepathget_buf_output: buf_output CSVファイルの絶対パスを取得",
        "- get_system_info: システム情報を一括取得（このスキル）",
        "- set_pad_override: PAD Overrideを有効・無効にする",
        "- set_pad_values: ボタン名・左右スティック・L2/R2トリガーの値を設定する",
        "- check_logic_cartridge_connection: logic/Logic_cartridge.pyの読み込みと関数の動作を確認する",
        "",
    ])
