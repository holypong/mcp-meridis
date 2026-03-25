////////////////////////////////////////////////////////////////////////////////
//	UVC (Universal Vibratory Controller) 歩行制御ライブラリ
//	二足歩行ロボット用の高度な歩行・バランス制御システム
//	
//	Base Algorithm:
//		https://ai2001.ifdef.jp
//		https://ai2001.ifdef.jp/uvc/QandA_Eng.html
//	+Hori 20231126 Created
////////////////////////////////////////////////////////////////////////////////
#include "config.h"
#include "atm_sensor.h"		// センサー関連の型定義を先に読み込み
#include <cstdint>			// 固定幅整数型の使用
#include "Arduino.h"		// Arduino標準ライブラリ（Serial出力用）
#include "math.h"			// 数学関数ライブラリ（三角関数・平方根等）

////////////////////////////////////////////////////////////////////////////////
//	基本定数・ID定義
////////////////////////////////////////////////////////////////////////////////

//	脚部識別ID
#define ID_CENTER   0			    // 中心基準点ID
#define ID_LEG_R    0			    // 右脚ID（軸足判定用）
#define ID_LEG_L    1			    // 左脚ID（軸足判定用）

//	デバッグ監視用ビットフラグ定義
#define MONITOR_TYPE_DEF    0x0000	// デフォルト（監視なし）
#define MONITOR_TYPE_MOTCT  0x0001	// 初期姿勢移行カウンタ
#define MONITOR_TYPE_IP     0x0002	// 角度補正処理カウンタ
#define MONITOR_TYPE_FWCT   0x0004	// 歩行周期カウンタ
#define MONITOR_TYPE_SW     0x0010	// 横振り移動量
#define MONITOR_TYPE_HIP    0x0020	// 腰高さ制御
#define MONITOR_TYPE_OFS    0x0040	// 角度補正オフセット値
#define MONITOR_TYPE_FEET   0x0080	// 左右足位置データ
#define MONITOR_TYPE_PRM_W  0x0100	// 歩容パラメータ

//  サーボID定義
#define ID_HIP_Y            0   // 股ヨー
#define ID_HIP_R            1   // 股ロール
#define ID_HIP_P            2   // 股ピッチ
#define ID_KNEE_P           3   // 膝ピッチ
#define ID_ANKLE_P          4   // 足首ピッチ
#define ID_ANKLE_R          5   // 足首ロール
#define ID_SHOULDER_P       6   // 肩ピッチ
#define ID_SHOULDER_R       7   // 肩ロール
#define ID_SHOULDER_Y       8   // 肩ヨー
#define ID_ELBOW_P          9   // 肘ピッチ
#define ID_HEAD_Y           10  // 頭部ヨー
#define ID_WAIST_Y          11  // 腰部ヨー

//  監視制御フラグ
#define MONITOR_FLOW_UVC    0	    // UVC制御フロー表示 0:OFF 1:ON
#define MONITOR_SET_RP_DEG  1       // 角度 Roll,Pitch 0:OFF(rad) 1:ON(deg)

//  UVC制御の初期設定値（set_status()初回コール時に適用）
#define UVC_START_AVOID_FALL_CFG  1		// 転倒回避機能 0:無効 1:有効
#define UVC_START_UVCW_CFG        1		// UVC歩行機能 0:無効 1:有効  
#define UVC_START_MIX_CFG         0		// ジャイロフィードバック 0:無効 1:有効

// RPY角度補正タイプ
#define RPY_ADJ_TYPE_SIMPLE 0               // 0:単純・外乱に弱い・高速で終了 
#define RPY_ADJ_TYPE_HIRES  1               // 1:高精度・外乱に強い・不安定が持続すると遅延
#define RPY_ADJ_TYPE        RPY_ADJ_TYPE_SIMPLE   // RPY角度補正タイプ


// 基本パラメータ定数（寸法、スペック）
#define HIPP_ANKP           129.0           // 股関節ピッチから足首ピッチまでの初期距離(mm) - 膝カクつき防止の安全値
#define HIP_LEG             21.5            // 腰中心から股関節ロールまでの距離(mm)

#define LIMIT_SW_LEG_XY     70.0            // 遊脚の可動範囲制限XY(mm)
#define LIMIT_SHOULDER_ROLL 60.0 /180 *M_PI // 肩ロール最大可動角度(rad)
#define LIMIT_WAIST_YAW     6.0  /180 *M_PI // 腰部ヨー最大可動角度(rad)
#define LIMIT_KNEE_PITCH    60.0 /180 *M_PI // 60度をラジアンに変換
#define LIMIT_SW_LEG_INSIDE 5.0             // 足の内側最大可動距離(mm) 15->5

#define HIP_H_START         200.0           // スタート時の脚長
#define HIP_H_WALK          175.0           // 標準脚長(mm) 地面から股関節までの高さ 185
#define HIP_H_MIN           140.0           // 最低脚長補償
#define HIP_X_OFFSET        0.0             // 腰重心前後オフセット(mm) - 支持脚前方移動量
#define STRIDE_MAX          45.0            // 最大歩幅(mm) 45.0
#define TURN_MAX            45.0            // 最大旋回角度(deg)
#define STRIDE_MID          STRIDE_MAX * 0.66   // 中間歩幅(mm) 30
#define WALK_STRIDE_FB      STRIDE_MAX * 0.33   // 中間歩幅(mm) UVC時前後の抑制 15
#define WALK_STRIDE_LR      STRIDE_MAX * 0.33   // 中間歩幅(mm) UVC時左右の抑制 15
#define WALK_TURN_LR        TURN_MAX * 0.33   // 中間旋回(deg) UVC時旋回の抑制 15

#define UVCW_SHOULDER_ROLL  10.0 /180 *M_PI // UVC歩行時の肩ロール角度(rad)

#define VEL_RETURN_LEG      1.5/180 *M_PI   // 軸足垂直復帰率：軸足を垂直に戻す速度 1.5度/制御周期
#define VEL_J_MAX           4.0/180 *M_PI   // 最大回転速度： 4.0度/制御周期（少し抑えて）
                                            //  KRS-2552RHV性能は 0.14秒/60度
                                            //  秒間 428.57度、約4.29度/制御周期

// RPY角度制御パラメータ
#define RPY_ALLOW           2.0 /180 *M_PI  // RPY検出精度許容値(deg->rad) 1.9
// Base code(BNO055) set "1" value, "1" means 1 / 16 deg = 0.0625 deg
// This code(MPU6050) can get with acculacy of 0.01 deg.

#define UVCW_ROLL_THRES     2.0 /180 *M_PI  // UVC歩行時のロール角度閾値(deg->rad) 1.9
#define UVCW_PITCH_THRES    3.0 /180 *M_PI  // UVC歩行時のピッチ角度閾値(deg->rad) 2.5
#define UVCW_RPY_THRES      3.0 /180 *M_PI  // UVC歩行時のRPY角度閾値(deg->rad) 3.0

#define UVC_ROLL_THRES      4.0 /180 *M_PI  // UVC制御時のロール角度閾値(deg->rad) 4.0
#define UVC_PITCH_THRES     6.0 /180 *M_PI  // UVC制御時のピッチ角度閾値(deg->rad) 4.0
#define UVC_RPY_THRES       6.0 /180 *M_PI  // UVC制御時のRPY角度閾値(deg->rad) 6.0

#define DOWN_ROLL_THRES     30.0 /180 *M_PI // 転倒のロール角度閾値(deg->rad) 20.0
#define DOWN_PITCH_THRES    30.0 /180 *M_PI // 転倒のピッチ角度閾値(deg->rad) 20.0

#define ANALOG_STICK_THRES  10.0/127         // アナログスティック無効閾値(-127 to 127) 5.0->10.0

////////////////////////////////////////////////////////////////////////////////
//	歩行制御ステートマシン定義
//	状態遷移: M0 → M710 → M720 → M730 ⇄ (M740 → M750 → M760 → M770) → M730
//	UVC制御: M740でUpperbody Vertical Control（上半身垂直制御）を実行
////////////////////////////////////////////////////////////////////////////////
#define M0     0		// 待機状態：       PADのキー入力待ち・停止状態での待機
#define M710   1		// 初期姿勢移行：   歩行準備のため腰を下げて重心安定化
#define M720   2		// 角度補正：       IMUセンサーの傾斜角度をゼロ基準に補正
#define M730   3		// 歩行か静止か：    歩行動作または静止動作の選択・実行
#define M740   4		// UVC制御：        上半身垂直制御による転倒防止・姿勢復帰
#define M750   5		// 振動減衰1：      UVC制御後の姿勢振動収束待機
#define M760   6		// 回復動作：       安全確認のための慎重歩行（短周期モード）
#define M770   7		// 振動減衰2：      完全安定確認後の通常歩行M730復帰準備
#define M780   8		// 予備状態1：      （将来拡張用・未使用）
#define M790   9		// 予備状態2：      （将来拡張用・未使用）
#define M791   10		// 予備状態3：      （将来拡張用・未使用）
#define M700   11		// デバッグ状態：   開発用モニタリング・診断モード

// 歩行制御パラメータ定数（もっとも基本的な設定）
#define CNT_FWCT_END            36          // 歩行周期終了設定値 36*10ms=360ms
#define SWING_MAX               24.0        // 脚振り最大設定値 24mm
#define FOOT_H_MAX              35.0        // 脚高さ最大設定値 20mm

#define CNT_FWCT_END_UVCW       30          // 歩行周期終了設定値(歩行) 48*10ms=480ms -> 30*10ms=300ms
#define SWING_MAX_UVCW          24.0        // 脚振り最大設定値 12mm
#define FOOT_H_MAX_UVCW         20.0        // 脚高さ最大設定値 20mm
#define FOOT_FWD_UVCW           10.0        // 足前方最大移動距離 10mm
#define FOOT_SIDE_UVCW          10.0       // 足前方最大移動距離 10mm

// カウンタ系
#define CNT_INIT_POSE           0           // 初期姿勢移行期間 48*10ms=480ms 0でスキップ 48->0
#define CNT_OFFSET_MAX          1           // 角度補正期間　100*10ms=1秒 1で最短 100->1
#define CNT_RETURN_LEG          25		    // 軸足垂直復帰率：軸足を垂直に戻す期間 25*10ms=250ms
#define CNT_UVC_DAMPING         20          // UVC制御後の振動減衰期間 30*10ms=300ms 20->30
#define CNT_RETURN_DAMPING      30          // 復元後の振動減衰期間 50*10ms=500ms 50->30

// 期間系（割合）
/*
#define TERM_FOOT_LAND          0.05        // 足上げ制御：着地期間の歩行周期に対する割合
#define TERM_FOOT_WEIGHT_SHIFT  0.25        // 足上げ制御：重心移動期間の歩行周期に対する割合
#define TERM_FOOT_UP            0.70        // 足上げ制御：脚上げ期間の歩行周期に対する割合（不使用）
#define TERM_FOOT_PREPARE_LAND  0.05        // 足上げ制御：着地準備期間の歩行周期に対する割合（不使用）
#define TERM_HIP_WEIGHT_SHIFT   0.3         // 横振り制御：重心移動期間の歩行周期に対する割合
#define TERM_LAND_STRIDE        0.1         // 前進制御：着地期間の歩行周期に対する割合
*/
#define TERM_FOOT_LAND          0.10        // 足上げ制御：着地期間の歩行周期に対する割合
#define TERM_FOOT_WEIGHT_SHIFT  0.35        // 足上げ制御：重心移動期間の歩行周期に対する割合
#define TERM_FOOT_UP            0.70        // 足上げ制御：脚上げ期間の歩行周期に対する割合（不使用）
#define TERM_FOOT_PREPARE_LAND  0.05        // 足上げ制御：着地準備期間の歩行周期に対する割合（不使用）
#define TERM_HIP_WEIGHT_SHIFT   0.40         // 横振り制御：重心移動期間の歩行周期に対する割合
#define TERM_LAND_STRIDE        0.1         // 前進制御：着地期間の歩行周期に対する割合


// 係数系
#define G_FOOT_H_MAX            0.3         // 足上げ制御：着地準備期間での足高さ制限係数
#define G_SW_XY_MID             0.5         // 横振り制御：中間歩幅設定係数
#define G_SWING_MID             0.7         // 脚振り中間設定値 SWING_MAX 25 * 0.7 = 17mm
#define G_RET_LEG_CTRL          0.07        // 軸足復帰制御：軸足を元の位置に戻す速度
#define G_ABS_TOUCH_CTRL        0.02        // 接地位置固定制御：接地位置を固定する速度
#define G_IMU_FEEDBACK          0.25        // IMUフィードバック制御：傾斜角度補正の係数
#define G_UVC_DECAY_FACTOR      0.80        // UVC積分減衰係数 core.cpp 0.9
#define G_ROLL_CORRECTION       0.50        // ロール角補正係数 core.cpp 1.5 -> 0.5
#define G_PITCH_CORRECTION      0.25        // ピッチ角補正係数 core.cpp 1.5 -> 0.25
#define G_LATERAL_SWING         1.20        // 横振り歩行時の横振り増強係数 1.0

class UVCClass
{
    public:
    // ■ 制御機能の有効/無効フラグ
    bool    uvc_avoid_fall_enable = false;  // 転倒回避機能 0:無効 1:有効
    bool    uvc_walk_enable       = false;  // UVC歩行機能 0:無効 1:有効
    bool    mix_enable            = false;  // ジャイロフィードバック 0:無効 1:有効

    // UVC制御実行中フラグ
    bool    doing = false;              // UVC制御実行中フラグ 0:停止 1:実行中
    bool    walk_on = false;            // 歩行実行中フラグ - ジョイスティック前進で開始


    // ■ ロボット関節角度データ（度単位）
    float   joint_l[15] = {};           // 左半身15関節の目標角度配列
    float   joint_r[15] = {};           // 右半身15関節の目標角度配列
    int     joint_cmd_l[15] = {};       // 左半身15関節のトルクON/OFF指令
    int     joint_cmd_r[15] = {};       // 右半身15関節のトルクON/OFF指令
    bool    fallDown = false;           // 転倒検知フラグ - 転倒時true
    
    // ■ 基準姿勢保存用配列（パッドボタン操作時の復帰用）
    float   base_l[15] = {};            // 左半身基準姿勢保存配列
    float   base_r[15] = {};            // 右半身基準姿勢保存配列
    bool    base_saved = false;         // 基準姿勢保存済みフラグ
    
    int16_t jikuasi;                    // 軸足判定 0:右足軸 1:左足軸（歩行の基準脚）
    int16_t mode;                       // 状態管理：現在/次/サブ

    vector3 foot_l;                     // 左足位置ベクトル
    vector3 foot_r;                     // 右足位置ベクトル

    vector3 foot_direction;           // 基準位置ベクトル
    vector3 r_foot;                     // 右足位置ベクトル
    vector3 l_foot;                     // 左足位置ベクトル
    float landing_start;                // 着地開始
    float landing_end;                  // 着地終了

    private:

    // +UVC 20220822 from core.cpp
    // ■ 関節角度制御用配列 [0]=右側 [1]=左側
    float TGT[12][2];	                // 目標関節角度格納用配列

    // ■ 歩行制御の中核変数

    int16_t motCt;                      // モーション制御カウンタ群
    int16_t	walkCt;		                // 歩数カウンタ/上限値

    int16_t	land_front,land_back;       // 着地制御パラメータ：前後（着地・離陸）

    int16_t	integ_cnt;                  // 角度補正用カウンタ

    rpy3    integ_a;                    // 角度補正用積算値構造体
    rpy3    integ_b;                    // 角度補正用積算値構造体バックアップ

    // ■ 歩行周期制御の中核変数
    int8_t  fwct_end,fwct,fwctUp;       // 歩行周期：最大値/現在値/増加分

    // ■ 姿勢角度データ（ラジアン）
    rpy3    cur_rpy;                    // IMU現在角度: roll=横,pitch=縦
    rpy3    tgt_rpy;                    // 目標姿勢角度群（UVC制御用）

    rpy3    snsr_rpy;                   // センサー角度値群（補正前後）
    rpy3    snsr_rpy_offset;            // センサーオフセット値群    
    rpy3    snsr_gyr;                   // センサー角度値群（補正前後）
    rpy3    snsr_acc;                   // センサー加速度値群

    // ■ 歩行制御の位置・姿勢変数（変数名の語源と物理的意味）
    float   wk,wt;                      // w=waist 腰
    float   fw,sw;                      // 歩行制御補助: fw=forward, sw=swing
    float   sw_max;                     // 脚振り max=最大値
    float   hip_h;                      // 腰高さ自動調整量 上下方向
    float   foot_h;                     // 脚高さ h=上方向
    float   foot_h_max;                 // 脚高さ h=上方向、max=最大値

    vector3 sw_sub;                     // 脚振り x=前後成分, y=左右成分

    vector3 sp_leg;                     // 支持脚位置ベクトル
    vector3 sp_leg_b;                   // 支持脚位置ベクトル backup
    vector3 sw_leg;                     // 遊脚位置ベクトル
    
    int8_t  fwct_end_set;               // 歩行周期の設定値（基準カウンタ）
    float   foot_h_max_set,sw_max_set;	// 脚上げ高さ最大値、横振り幅最大値の設定

    // sensor and joystick analog feedback +Hori 20231221
    short mv_mix_r[9][15] = {
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -8, 0, 0, 0, 0},  // 0 gyro_roll
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  8,  0, 0, 0, 0, 0},  // 1 gyro_pitch
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 2 gyro_yaw
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 3 joy_l_x
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 4 joy_l_y
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 5 joy_l_z
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 6 joy_r_x
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 7 joy_r_y
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0}}; // 8 joy_r_z

    short mv_mix_l[9][15] = {
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  8, 0, 0, 0, 0},  // 0 gyro_roll
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  8,  0, 0, 0, 0, 0},  // 1 gyro_pitch
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 2 gyro_yaw
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 3 joy_l_x
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 4 joy_l_y
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 5 joy_l_z
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 6 joy_r_x
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0},  // 7 joy_r_y
            {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, 0, 0}}; // 8 joy_r_z


    /////////////////////////////////////
    //// 目標値まで徐々に間接を動かす ////
    /////////////////////////////////////
    // Note: 初期姿勢を直接与えるときに使用
    //       急ぎの場合は、使用しない
    void move_servo(short *cur ,int dst){
    //引数 s:現サーボ位置 d:目標サーボ位置
        if(motCt<1)
            *cur = dst;
        else
        	*cur += (dst - *cur)/motCt;
    }

    ///////////////////////////
    //// 検出角度を校正する ////
    ///////////////////////////
    // Note: 積分平均で角度補正を行う方式
    //       急ぎの場合は、1回だけの補正で終了する
    //       M720でのみ実行する機能
    void angle_adjust_imu(){

        // **** pitch & roll ****
        if( RPY_ADJ_TYPE == RPY_ADJ_TYPE_SIMPLE){           // 単純方式
            ++integ_cnt;
            integ_a.pitch += snsr_rpy.pitch;
            integ_a.roll += snsr_rpy.roll;

        }else if( RPY_ADJ_TYPE == RPY_ADJ_TYPE_HIRES){      // 高精度方式
            if( snsr_rpy.pitch <= integ_b.pitch+RPY_ALLOW && snsr_rpy.pitch >= integ_b.pitch - RPY_ALLOW &&
                snsr_rpy.roll <= integ_b.roll+RPY_ALLOW && snsr_rpy.roll >= integ_b.roll - RPY_ALLOW 	){
                ++integ_cnt;
                integ_a.pitch += snsr_rpy.pitch;
                integ_a.roll += snsr_rpy.roll;
            }
            else {
                integ_cnt = 0;
                integ_a.pitch = 0;
                integ_a.roll = 0;
            }
            integ_b.pitch = snsr_rpy.pitch;
            integ_b.roll = snsr_rpy.roll;
        }

        flow_monitor(MONITOR_TYPE_IP);
    }


    ///////////////////////
    //// 全方向転倒検知 ////
    ///////////////////////
    // Note: ロールが閾値内ANDピッチが閾値内なら転倒していないと判断
    //       どちらか一方でも閾値を超えたら転倒と判断し、転倒処理を実行する
    //       急ぎの場合は、転倒を検知しても転倒準備姿勢をとらずに振り出しに戻る
    //       各modeの最後に呼び出される
    void detection_angle_falldown(){

        // 転倒していない場合はこの処理を抜ける
        if( DOWN_PITCH_THRES > fabs(cur_rpy.pitch) && DOWN_ROLL_THRES > fabs(cur_rpy.roll) ) 
            return;

        flow_monitor(MONITOR_TYPE_DEF);

        // 転倒準備姿勢をとって脱力することで転倒ダメージを軽減する
        // 検証の邪魔になるときはコメントアウトする
        /*
        // 初期姿勢をセット
        TGT[ID_HEAD_Y][ID_CENTER]   = 0.0;      //頭ヨー
        TGT[ID_WAIST_Y][ID_CENTER]  = 0.0;      //腰ヨー
        for(int i=0; i<2; i++){
            TGT[ID_SHOULDER_P][i]   = -13.5 / 180.0 * M_PI;	//肩ピッチ
            TGT[ID_SHOULDER_R][i]   = 0.0;      //肩ロール
            TGT[ID_SHOULDER_Y][i]   = 0.0;	    //肩ヨー
            TGT[ID_ELBOW_P][i]      = -27.0 / 180.0 * M_PI;	//肘ピッチ
            TGT[ID_HIP_Y][i]        = 0.0;	    //股ヨー
            TGT[ID_HIP_R][i]        = 0.0;	    //股ロール
            TGT[ID_HIP_P][i]        = -21.0 / 180.0 * M_PI;	//股ピッチ
            TGT[ID_KNEE_P][i]       = 42.0  / 180.0 * M_PI;	//膝ピッチ
            TGT[ID_ANKLE_P][i]      = -21.0 / 180.0 * M_PI;	//足首ピッチ
            TGT[ID_ANKLE_R][i]      = 0.0;      //足首ロール
        }
        // 脱力
        for(int i=0;i<15;i++){
            joint_cmd_l[i] = 0;
            joint_cmd_r[i] = 0;
        }
        */

        mode = M0;          // 待機状態へ移行
        fallDown = true;    // 転倒検知フラグセット

        //Serial.print("turn_over!!");
        //Serial.println();
	    //while(1){}

    }

    /////////////////////
    //// UVC補助制御 ////
    /////////////////////
	// UVC補助制御処理1
	// UVC補助制御処理1 - 上半身垂直化初期制御
	// 機能: 上半身垂直制御後の初期段階での脚配置正規化
	//       - 着地前期間での軸足の垂直下方配置への復帰
	//       - 左右方向: 軸足を体幹中央下に戻し、遊脚を安定配置
	//       - 前後方向: 軸足を体幹重心下に段階的復帰
	//       - 股関節高さの着地準備・衝撃吸収制御
	// 目的: 上半身垂直化制御で変位した脚位置の段階的正規化
	// 対象期間: fwct ≤ land_front（歩行周期の前半部分）
	// 実行: mode M740（UVC上半身垂直制御動作中）で呼び出される
    void uvc_sub_1(){
        float k;

        // ************ UVC終了時軸足を垂直に戻す ************
        if( fwct <= land_front ){

            // **** 左右方向 ****
            k = sp_leg.y/(CNT_RETURN_LEG/2 - fwct); // 直値11を25/2で近似 +-Hori 20251017
            sp_leg.y -= k;
            sw_leg.y += k;

            // **** 前後方向 ****
            if( sp_leg.x > VEL_RETURN_LEG ){
                sp_leg.x -= VEL_RETURN_LEG;
                sw_leg.x -= VEL_RETURN_LEG;
            }
            else if( sp_leg.x < -VEL_RETURN_LEG ){
                sp_leg.x += VEL_RETURN_LEG;
                sw_leg.x += VEL_RETURN_LEG;
            }
            else{
                sw_leg.x -= sp_leg.x;
                sp_leg.x  = 0;
            }
        }
        if(sw_leg.y >  LIMIT_SW_LEG_XY)	sw_leg.y=  LIMIT_SW_LEG_XY;
        if(sw_leg.x < -LIMIT_SW_LEG_XY)	sw_leg.x= -LIMIT_SW_LEG_XY;
        if(sw_leg.x >  LIMIT_SW_LEG_XY)	sw_leg.x=  LIMIT_SW_LEG_XY;

        // ************ 脚長制御 ************
        if(HIP_H_WALK > hip_h){						        //脚長を徐々に復帰させる
            hip_h +=(float)((HIP_H_WALK - hip_h)*G_RET_LEG_CTRL);
        }
        else
            hip_h = HIP_H_WALK;

        if( fwct > (fwct_end - land_back) && tgt_rpy.roll > 0){	//着地時姿勢を落し衝撃吸収
            hip_h -=(float)(( fabs(sp_leg.y - sp_leg_b.y) + fabs(sp_leg.x - sp_leg_b.x) ) * G_ABS_TOUCH_CTRL);
        }
        if(HIP_H_MIN > hip_h) 
            hip_h = HIP_H_MIN;	                            //最低脚長補償
    }

	// UVC補助制御処理2 - 上半身垂直化完了制御
	// 機能: 上半身垂直制御後の最終段階での標準姿勢完全復帰
	//       - 軸足位置の残存偏差を段階的に標準位置に収束
	//       - 左右・前後方向の線形復帰制御（体幹中央下配置）
	//       - 腰回転角度の段階的中央復帰（垂直軸回転の正規化）
	//       - 遊脚位置の軸足変化に追従した適応調整
	//       - 股関節高さの標準歩行高さへの線形復帰
	// 目的: 上半身垂直化制御完了後の通常歩行姿勢への完全復帰
	// 復帰方式: 残り時間比例による段階的減衰制御
	// 実行: mode M740（UVC上半身垂直制御動作の後半）で呼び出される
    void uvc_sub_2(){
        float k0,k1;

        // ************ UVC終了時軸足を垂直に戻す ************

        // **** 左右方向 ****
        k1 = sp_leg.y/(fwct_end - fwct +1);
        sp_leg.y -= k1;

        // **** 前後方向 ****
        k0 = sp_leg.x/(fwct_end - fwct +1);
        sp_leg.x  -= k0;

        // **** 腰回転 ****
        wk -= wk/(fwct_end - fwct+1);
        TGT[ID_WAIST_Y][ID_CENTER]  -= (float)(TGT[ID_WAIST_Y][ID_CENTER]/(fwct_end-fwct+1));
        TGT[ID_HIP_Y][ID_LEG_R]  = (float)( TGT[ID_WAIST_Y][ID_CENTER]);
        TGT[ID_HIP_Y][ID_LEG_L]  = (float)(-TGT[ID_WAIST_Y][ID_CENTER]);

        if( fwct<=land_front ){
            // **** 左右方向 ****
            sw_leg.y += k1;

            // **** 前後方向 ****
            sw_leg.x -= k0;
        }
        else{
            sw_leg.y -= sw_leg.y/(fwct_end - fwct +1);
            sw_leg.x -= sw_leg.x/(fwct_end - fwct +1);
        }

        // **** 脚長制御 ****
        hip_h += (HIP_H_WALK - hip_h)/(fwct_end - fwct +1);

        if(sw_leg.y> LIMIT_SW_LEG_XY)	sw_leg.y=  LIMIT_SW_LEG_XY;
        if(sw_leg.x<-LIMIT_SW_LEG_XY)	sw_leg.x= -LIMIT_SW_LEG_XY;
        if(sw_leg.x> LIMIT_SW_LEG_XY)	sw_leg.x=  LIMIT_SW_LEG_XY;
    }


    /////////////////
    //// UVC制御 ////
    /////////////////
    // UVC（Upperbody Vertical Control：上半身垂直制御）のメインルーチン
    // 機能: 姿勢センサー情報に基づく上半身垂直維持制御
    //       - 傾斜角合成による閾値判定と垂直復帰補正開始
    //       - ロール角による左右方向の脚配置調整（上半身垂直化）
    //       - ピッチ角による前後方向の脚配置調整（上半身垂直化）
    //       - 軸足・遊脚の歩幅制限処理
    // 目的: 傾斜した上半身を垂直状態に復帰させる転倒防止制御
    // 実行: mode M740（転倒防止動作）で呼び出される
    void uvc(){
        float pb,rb,k,k_d,ks,kl;

        // ************ 傾斜角へのオフセット適用 ************
        rb = cur_rpy.roll;		//一時退避
        pb = cur_rpy.pitch;
        k = (float)sqrt(cur_rpy.pitch*cur_rpy.pitch + cur_rpy.roll*cur_rpy.roll);	//合成傾斜角
        

        // ロールピッチの合成傾斜角が閾値を超えたらUVC補正を開始する
        // 合成傾斜角に対する閾値超過分の比を求め、各軸に乗じて仮想現在角とする
        if( k > UVC_RPY_THRES ){
            k_d = (float)(k - UVC_RPY_THRES)/k; //閾値超過分の比を求める
            cur_rpy.pitch *= k_d;               //仮想現在角(pitch)
            cur_rpy.roll  *= k_d;               //仮想現在角(roll)
        }
        else{
            // 閾値以下なら補正しないとする（値を一時的に0にする）
            cur_rpy.pitch = 0;                  //仮想現在角(pitch)
            cur_rpy.roll  = 0;                  //仮想現在角(roll)
        }

        // ************ 傾斜角に係数適用 ************
        // 目標角度は、仮想傾斜角にフィードバックゲインを乗じた値
        tgt_rpy.roll = (float) G_IMU_FEEDBACK * cur_rpy.roll;   //ロール目標角を求める  
        if(jikuasi == ID_LEG_R) tgt_rpy.roll = -tgt_rpy.roll;   //右軸足ならロール軸は反転
        
        tgt_rpy.pitch = (float) G_IMU_FEEDBACK * cur_rpy.pitch; //ピッチ目標角を求める

        // 着地・離陸制御を除きUVC制御を実行
        if(fwct > land_front && fwct <= fwct_end - land_back ){

            // ************ UVC主計算 ************
            // 支持脚位置を計算する
            k  = (float)atan((sp_leg.y - sw)/hip_h );	        //片脚の鉛直に対する開脚角
            kl = (float)(hip_h / cos(k));		                //前から見た脚投影長
            ks = k + tgt_rpy.roll;					            //開脚角に横傾き角を加算
            k  = (float)(kl * sin(ks));			                //中点から横接地点までの左右距離
            sp_leg.y	  = k + sw;					            //横方向UVC補正距離
            hip_h = (float)(kl * cos(ks));				        //K1までの高さ更新

            // **** UVC（前後） *****
            k 	  = (float)atan( sp_leg.x / hip_h );	        //片脚のX駆動面鉛直から見た現時点の前後開脚角
            kl	  = (float)(hip_h / cos(k));		            //片脚のX駆動面鉛直から見た脚長
            ks	  = k + tgt_rpy.pitch;					        //振出角に前後傾き角を加算
            k	  = (float)(kl * sin(ks));		                //前後方向UVC補正距離
            sp_leg.x = k;						                //前後方向UVC補正距離
            hip_h = (float)(kl * cos(ks));		                //K1までの高さ更新

            // ************ UVC積分値リミット設定 ************
            if(sp_leg.y <  0)		    sp_leg.y =  0;
            if(sp_leg.y >  STRIDE_MAX)	sp_leg.y =  STRIDE_MAX;
            if(sp_leg.x < -STRIDE_MAX)	sp_leg.x = -STRIDE_MAX;
            if(sp_leg.x >  STRIDE_MAX)	sp_leg.x =  STRIDE_MAX;

            // ************ 遊脚側を追従させる ************
            // 遊脚位置を支持脚位置の逆方向に設定する
            sw_leg.y = sp_leg.y;			//遊脚Y目標値
            sw_leg.x = -sp_leg.x;			//遊脚X目標値

            // ************ 両脚内側並行補正 ************
            // 両脚が内側に並行以下にならないように補正する
            if(jikuasi == ID_LEG_R){		    //両脚を並行以下にしない
                k  = -sw + sp_leg.y;	            //右足の外側開き具合
                ks =  sw + sw_leg.y;	            //左足の外側開き具合
            }
            else{
                ks = -sw + sp_leg.y;	            //左足の外側開き具合
                k  =  sw + sw_leg.y;	            //右足の外側開き具合
            }
            if(k + ks < 0) sw_leg.y -= k + ks;  //遊脚を平衡に補正
        }

        // 一時退避値を元に戻す
        cur_rpy.roll  = rb;
        cur_rpy.pitch = pb;
    }

    // 目的: 傾斜した上半身を垂直状態に復帰させる転倒防止制御（歩行時用）
    //       - core.cpp版UVCの歩行特化型簡易版
    // 実行: mode M740（転倒防止動作）で呼び出される
    void uvcw(){
        float pb,rb,k,k_d;

        // ************ 傾斜角へのオフセット適用 ************
        rb = cur_rpy.roll;		//一時退避
        pb = cur_rpy.pitch;
        k = (float)sqrt(cur_rpy.pitch*cur_rpy.pitch + cur_rpy.roll*cur_rpy.roll);	//合成傾斜角
        
        // ロールピッチの合成傾斜角が閾値を超えたらUVC補正を開始する
        // 合成傾斜角に対する閾値超過分の比を求め、各軸に乗じて仮想現在角とする
        if( k > UVCW_RPY_THRES ){
            k_d = (float)(k - UVCW_RPY_THRES)/k; //閾値超過分の比を求める
            cur_rpy.pitch *= k_d;               //仮想現在角(pitch)
            cur_rpy.roll  *= k_d;               //仮想現在角(roll)
        }
        else{
            // 閾値以下なら補正しないとする（値を一時的に0にする）
            cur_rpy.pitch = 0;                  //仮想現在角(pitch)
            cur_rpy.roll  = 0;                  //仮想現在角(roll)
        }

        // 着地・離陸制御を除きUVC制御を実行
        if(fwct > land_front && fwct <= fwct_end - land_back ){

            // UVC主計算（シンプル版）
            // ロール角による左右方向補正（振動抑制版）
            k = G_ROLL_CORRECTION  * HIP_H_WALK * sin(cur_rpy.roll);    // 係数で振動を抑制
            if(jikuasi == ID_LEG_R)  // 右足が軸足の場合
                sp_leg.y += k;          // 軸足Y位置に補正加算
            else                     // 左足が軸足の場合
                sp_leg.y -= k;          // 軸足Y位置に補正減算
            // ピッチ角による前後方向補正（振動抑制版）
            k = G_PITCH_CORRECTION * HIP_H_WALK * sin(cur_rpy.pitch);   // 係数で振動を抑制
            sp_leg.x += k;       // 軸足X位置に補正加算

            sp_leg.y *= G_UVC_DECAY_FACTOR;
            sp_leg.x *= G_UVC_DECAY_FACTOR;

            // ************ UVC積分値リミット設定 ************
            if(sp_leg.y <  0)		    sp_leg.y =  0;
            if(sp_leg.y >  STRIDE_MID)	sp_leg.y =  STRIDE_MID;
            if(sp_leg.x < -STRIDE_MID)	sp_leg.x = -STRIDE_MID;
            if(sp_leg.x >  STRIDE_MID)	sp_leg.x =  STRIDE_MID;

            // ************ 遊脚側を追従させる ************
            // 遊脚位置を支持脚位置の逆方向に設定する
            sw_leg.y = sp_leg.y;			//遊脚Y目標値
            sw_leg.x = -sp_leg.x;			//遊脚X目標値

        }

        // 一時退避値を元に戻す
        cur_rpy.roll  = rb;
        cur_rpy.pitch = pb;
    }


    ////////////////////////////////////////////////////
    //// 脚上げ高さ制御 - 歩行周期に応じたSin曲線制御 ////
    ////////////////////////////////////////////////////

    void foot_up(){
        // ■ 歩行周期を4段階に分割した安定歩行制御
        // 着地期間 約5% → 重心移動 約25% → 脚上げ 約70% → 着地準備 約5%
        
        int landPeriod = (int)(fwct_end * TERM_FOOT_LAND);                   // 着地期間を元に戻す
        int weightShiftPeriod = (int)(fwct_end * TERM_FOOT_WEIGHT_SHIFT);    // 重心移動期間を元に戻す

        if(fwct <= landPeriod && landPeriod > 0){
            // ■ 第1段階：着地安定期間
            foot_h = 0;  // 遊脚を地面に確実に接地させ安定性確保
        }
        else if(fwct <= weightShiftPeriod){
            // ■ 第2段階：重心移動期間 軸足への体重移動
            foot_h = 0;  // 重心移動完了まで両足接地を維持
        }
        else if(fwct <= fwct_end - landPeriod){
            // ■ 第3段階：脚上げ期間 Sin曲線で滑らかな上下動
            float phase = M_PI * (fwct - weightShiftPeriod) / (fwct_end - weightShiftPeriod - landPeriod);
            foot_h = (float)(foot_h_max * sin(phase));                      // 0→最大→0の滑らかな軌道
        }
        else {
            // ■ 第4段階：着地準備期間（最後5%）ソフトランディング
            float phase = M_PI * (fwct_end - fwct) / (landPeriod + 1);      // ゼロ除算回避
            foot_h = (float)(foot_h_max * G_FOOT_H_MAX * sin(phase));     // 低めの高さで確実な着地
        }
    }

    ////////////////////////////////////////////////////////////
    //// 横振り制御 - 重心移動のための左右方向振り子制御 ////
    ////////////////////////////////////////////////////////////
    // +Hori 2023123 add gain
    void swing_control(){
        float k,t;                          // 計算用一時変数

        // ■ 2段階重心移動：前半30%で大移動、後半70%で標準制御
        float phase_ratio = (float)fwct / fwct_end;  // 歩行周期内の位置（0.0-1.0）
        
        if(phase_ratio < TERM_HIP_WEIGHT_SHIFT) {
            // ■ 前半30%：強化重心移動期間（支持脚側への大きな移動）
            k = (float)(sw_max * G_LATERAL_SWING * sin(M_PI * phase_ratio / TERM_HIP_WEIGHT_SHIFT));
        } else {
            // ■ 後半70%：標準Sin制御での滑らかな復帰
            k = (float)(sw_max * sin(M_PI * (phase_ratio - TERM_HIP_WEIGHT_SHIFT) / (1.0 - TERM_HIP_WEIGHT_SHIFT)));
        }
        
        // ■ 前後方向成分を考慮した横振り角度計算
        t=(float)(atan( (fabs(sp_leg.x) + fabs(HIP_LEG*sin(wt) ))/(sp_leg.y + HIP_LEG*cos(wt) -wt) ));

        if(sp_leg.x > 0)    // 前進時
            sw_sub.x =(float)( k*sin(t) );       // X成分（前後方向）
        else                // 後退時  
            sw_sub.x =(float)(-k*sin(t) );       // X成分符号反転

        sw_sub.y=(float)(k*cos(t));              // Y成分（左右方向）主成分

    }

    ////////////////
    //// 腕制御 ////
    ////////////////
    void arm_control(){
        TGT[ID_SHOULDER_R][ID_LEG_R]=(float)(LIMIT_SHOULDER_ROLL * sw_leg.y/LIMIT_SW_LEG_XY); //股幅に応じ腕を広げる

        if(TGT[ID_SHOULDER_R][ID_LEG_R] < 0) TGT[ID_SHOULDER_R][ID_LEG_R] = 0;

        TGT[ID_SHOULDER_R][ID_LEG_L] = TGT[ID_SHOULDER_R][ID_LEG_R];    //左右対称

    }

    ////////////////////
    //// 最終脚駆動 ////
    ////////////////////////////////////////////////////////////////
    ////	脚関節角度制御 - 目標位置から各関節の角度を逆運動学で計算	////
    ////////////////////////////////////////////////////////////////
    void foot_control(float x,float y,float h,int s){
    //	引数:
    //		x: 前後方向距離(mm) 前方=正　中心基準
    //		y: 左右方向距離(mm) 右側=正　中心基準  
    //		h: 地面から股関節までの高さ(mm) 最大194.5mm
    //		s: 脚ID 0=右脚 1=左脚
    //	処理:
    //		3D座標から膝・股・足首関節の目標角度を逆運動学計算
    //		物理制約（最大脚長129mm）による制限処理を含む

        float k;

        k = (float)(sqrt(x*x + pow(sqrt(y*y + h*h) - HIPP_ANKP/2, 2)));	//K0-A0間直線距離
        if(k>HIPP_ANKP){
            hip_h = (float)(sqrt(pow(sqrt(HIPP_ANKP*HIPP_ANKP - x*x) + HIPP_ANKP/2, 2) - y*y));//高さ補正
            k=HIPP_ANKP;
        }

        x = (float)(asin(x/k));						//K0脚振り角度
        k = (float)(acos(k/HIPP_ANKP));					//膝曲げ角度
        if(k>LIMIT_KNEE_PITCH) k=LIMIT_KNEE_PITCH;		//60°Max

        if		(2*k - TGT[ID_KNEE_P][s] >  VEL_J_MAX) k = (TGT[ID_KNEE_P][s] + VEL_J_MAX)/2;	//回転速度最大 0.13s/60deg = 138count
        else if	(2*k - TGT[ID_KNEE_P][s] < -VEL_J_MAX) k = (TGT[ID_KNEE_P][s] - VEL_J_MAX)/2;

        if(mode!=M0 && jikuasi == s)  // 軸足のたわみを考慮
            TGT[ID_KNEE_P][s]	= (float)(k*2);
        else
            TGT[ID_KNEE_P][s]	= (float)(k*2);

        TGT[ID_HIP_P][s]	= (float)(k + x);
        TGT[ID_ANKLE_P][s]	= (float)(k - x);

        k = (float)(atan(y/h));						//K1角度

        if		(k-TGT[ID_HIP_R][s] >  VEL_J_MAX) k = TGT[ID_HIP_R][s] +VEL_J_MAX;		        //回転速度最大 0.13s/60deg = 138count
        else if	(k-TGT[ID_HIP_R][s] < -VEL_J_MAX) k = TGT[ID_HIP_R][s] -VEL_J_MAX;

        if(mode!=M0 && jikuasi==s)  // 軸足のたわみを考慮
            TGT[ID_HIP_R][s] = (float)k;
        else
            TGT[ID_HIP_R][s] = (float)k;
        TGT[ID_ANKLE_R][s] = (float)(-k);
    }

    ////////////////////
    //// 統合脚駆動 ////
    ////////////////////
    void feet_control_1(float x0, float y0, float x1, float y1, int s){
    //x0:中点を0とする設置点前後方向距離（前+）：右足
    //y0:中点を0とする設置点左右方向距離（右+）：右足
    //x1:中点を0とする設置点前後方向距離（前+）：左足
    //y1:中点を0とする設置点左右方向距離（右+）：左足
    //s:腰部の制御 on/off、指定

        if(s == true){  // 腰部回転制御あり
            if(y0 + HIP_LEG == 0) // 0除算回避（股関節の付け根がベース中心上にある場合など）
            	wt = 0;             //腰回転角度初期化
            else if(jikuasi == ID_LEG_R){                       // 軸足が右
                wt = (float)(LIMIT_WAIST_YAW*atan( x0/(y0 + HIP_LEG) ));	//腰回転角度
                wk = (float)(fabs(LIMIT_SW_LEG_INSIDE*x0 / STRIDE_MAX));	//STRIDE_MAX:最大歩幅 LIMIT_STRIDE_INSIDE:足内側最大移動量
            }
            else{                                               // 軸足が左
                wt = (float)(LIMIT_WAIST_YAW*atan( -x1/(y1 + HIP_LEG) ));	//腰回転角度
                wk = (float)fabs(LIMIT_SW_LEG_INSIDE*x1 / STRIDE_MAX);      //STRIDE_MAX:最大歩幅 LIMIT_STRIDE_INSIDE:足内側最大移動量
            }

            // 腰を回転させた場合、股関節も連動させる
            TGT[ID_WAIST_Y][ID_CENTER] = (float)(wt);                       //腰回転（+で上体右回転）
            TGT[ID_HIP_Y][ID_LEG_R]  = (float)( TGT[ID_WAIST_Y][ID_CENTER]);
            TGT[ID_HIP_Y][ID_LEG_L]  = (float)(-TGT[ID_WAIST_Y][ID_CENTER]);
        }

        if(jikuasi == ID_LEG_R ){   // 軸足が右
            foot_control( x0,	y0 - wk,	hip_h,          ID_LEG_R );     // 軸足 右
            foot_control( x1,	y1 - wk,	hip_h - foot_h,	ID_LEG_L );     // 遊脚 左
        }
        else{                       // 軸足が左
            foot_control( x0,	y0 - wk,	hip_h - foot_h,	ID_LEG_R );     // 遊脚 右
            foot_control( x1,	y1 - wk,	hip_h,	        ID_LEG_L );     // 軸足 左
        }
    }

    ////////////////////
    //// 統合脚駆動 - 軸足/遊脚指定なし
    //s:腰部の制御 on/off、指定
    ////////////////////
    void feet_control_2(int s){
    
        // Z軸回転制御：股関節ヨー軸への回転角度反映        
        if(jikuasi == ID_LEG_R) {   // 軸足が右
            TGT[ID_HIP_Y][ID_LEG_R] = sp_leg.z / 180.0 * M_PI;  // 軸足（右足）にZ回転角を追加
            TGT[ID_HIP_Y][ID_LEG_L] = sw_leg.z / 180.0 * M_PI;  // 遊脚（左足）にZ回転角を追加
        } else {                    // 軸足が左
            TGT[ID_HIP_Y][ID_LEG_R] = sw_leg.z / 180.0 * M_PI;  // 遊脚（右足）にZ回転角を追加
            TGT[ID_HIP_Y][ID_LEG_L] = sp_leg.z / 180.0 * M_PI;  // 軸足（左足）にZ回転角を追加
        }

        // 軸足と遊脚に対して、足位置を補正して脚駆動を行う
        if(jikuasi == ID_LEG_R)     // 軸足が右
            feet_control_1( sp_leg.x - sw_sub.x, sp_leg.y - sw_sub.y, sw_leg.x - sw_sub.x, sw_leg.y + sw_sub.y ,s );  // 軸足 右のときの軸足/遊脚
        else                        // 軸足が左
            feet_control_1( sw_leg.x - sw_sub.x, sw_leg.y + sw_sub.y, sp_leg.x - sw_sub.x, sp_leg.y - sw_sub.y ,s );  // 軸足 左のときの軸足/遊脚
        
    }

    // 歩行中の足前後位置制御 x軸方向
    bool feet_direction_x(){
                        
        if(fwct < landing_start) {
            // 前期：0 → landing_start (足を閉じている期間)
            float prep_progress = fwct / landing_start;                                         // 前期進行度
            
            // 前後方向のみ変化
            sp_leg.x = foot_direction.x * prep_progress;
            
        } else if(fwct >= landing_start && fwct <= landing_end) {
            // 中期：着地期間の制御
            float landing_progress = (fwct - landing_start) / (landing_end - landing_start);    // 中期進行度
            
            // 前後方向：前→後への段階的変化
            sp_leg.x = foot_direction.x * (1.0 - 2.0 * landing_progress);
            
        } else {
            // 後期：landing_end → fwct_end (足を閉じる期間)
            float return_progress = (fwct - landing_end) / (fwct_end - landing_end);            // 後期進行度
            
            // 前後方向：後方位置から中央へ
            float linear_factor = (1.0 - return_progress);      // 線形変化
            sp_leg.x = -foot_direction.x + foot_direction.x * linear_factor * 0.5;
        }

        return true;
    }
    
    // 歩行中の足左右位置制御 y軸方向（最適化版）
    // foot_direction.yの正負から符号係数を決定し、条件分岐を最小化
    bool feet_direction_y(){

        r_foot = {0.0, 0.0, 0.0};
        l_foot = {0.0, 0.0, 0.0};

        // 移動指令なしは早期リターン
        if(foot_direction.y == 0.0) {
            return true;
        }

        // 方向に基づく定数計算
        // 右移動(+): dir_sign=+1, 第1サイクル軸足=左足, r_sign=+1, l_sign=+1
        // 左移動(-): dir_sign=-1, 第1サイクル軸足=右足, r_sign=+1, l_sign=-1
        float dir_sign = (foot_direction.y > 0) ? 1.0f : -1.0f;
        int first_leg = (foot_direction.y > 0) ? ID_LEG_L : ID_LEG_R;
        float abs_foot_dir_y = fabs(foot_direction.y);
        
        // 右足・左足への符号係数（左移動時のみ左足が負になる）
        float r_sign = 1.0f;
        float l_sign = dir_sign;

        // 第1サイクル判定
        bool is_first_cycle = (jikuasi == first_leg);

        // 期間別処理
        if(fwct < landing_start) {
            // 前期：足を閉じている期間（初期化済みの0.0のまま）
            
        } else if(fwct >= landing_start && fwct <= landing_end) {
            // 中期：着地期間
            float landing_progress = (fwct - landing_start) / (landing_end - landing_start);
            
            if(is_first_cycle && landing_progress > 0.5) {
                // 第1サイクル後半50%で両足を開く
                float opening_progress = (landing_progress - 0.5f) / 0.5f;
                r_foot.y = r_sign * abs_foot_dir_y * opening_progress;
                l_foot.y = l_sign * abs_foot_dir_y * opening_progress;
            }
            
        } else {
            // 後期：足を閉じる期間
            if(is_first_cycle) {
                float return_progress = (fwct - landing_end) / (fwct_end - landing_end);
                
                // 遊脚を閉じ、支持脚は維持
                if(foot_direction.y > 0) {  // 右移動：右足(遊脚)閉じる、左足(支持脚)維持
                    r_foot.y = abs_foot_dir_y * (1.0f - return_progress);
                    l_foot.y = abs_foot_dir_y;
                } else {  // 左移動：左足(遊脚)閉じる、右足(支持脚)維持
                    r_foot.y =  abs_foot_dir_y;
                    l_foot.y = -abs_foot_dir_y * (1.0f - return_progress);
                }
            }
        }

        return true;
    }

    // 歩行中の足旋回制御 z軸回転方向（最適化版）
    // foot_direction.zの正負から符号係数を決定し、条件分岐を最小化
    bool feet_direction_zw(){

        r_foot.z = 0.0;
        l_foot.z = 0.0;

        // 回転指令なしは早期リターン
        if(foot_direction.z == 0.0) {
            return true;
        }

        // 方向に基づく定数計算
        // 右回転(+): 第1サイクル軸足=左足, dir_sign=+1
        // 左回転(-): 第1サイクル軸足=右足, dir_sign=+1（符号を反転）
        float dir_sign = (foot_direction.z > 0) ? 1.0f : 1.0f;  // 左旋回時も正の符号
        int first_leg = (foot_direction.z > 0) ? ID_LEG_L : ID_LEG_R;
        int second_leg = (foot_direction.z > 0) ? ID_LEG_R : ID_LEG_L;
        float abs_foot_dir_z = fabs(foot_direction.z);
        
        // 右足・左足への符号係数（ミラー対象: 右足は常に正、左足は常に負）
        float r_sign =  1.0f;
        float l_sign = -1.0f;

        // サイクル判定
        bool is_first_cycle = (jikuasi == first_leg);
        bool is_second_cycle = (jikuasi == second_leg);

        // 期間別処理
        if(fwct < landing_start) {
            // 前期：第2サイクルのみ両足維持
            if(is_second_cycle) {
                r_foot.z =  r_sign * dir_sign * abs_foot_dir_z;
                l_foot.z =  l_sign * dir_sign * abs_foot_dir_z;
            }
            
        } else if(fwct >= landing_start && fwct <= landing_end) {
            // 中期：着地期間
            float landing_progress = (fwct - landing_start) / (landing_end - landing_start);
            
            if(is_first_cycle) {
                // 第1サイクル：後半50%で両足回転開始
                if(landing_progress > 0.5f) {
                    float opening_progress = (landing_progress - 0.5f) / 0.5f;
                    r_foot.z = r_sign * dir_sign * abs_foot_dir_z * opening_progress;
                    l_foot.z = l_sign * dir_sign * abs_foot_dir_z * opening_progress;
                }
            } else if(is_second_cycle) {
                // 第2サイクル：全期間で両足回転を徐々に減少
                float closing_progress = 1.0f - landing_progress;
                r_foot.z = r_sign * dir_sign * abs_foot_dir_z * closing_progress;
                l_foot.z = l_sign * dir_sign * abs_foot_dir_z * closing_progress;
            }
            
        } else {
            // 後期：第1サイクルのみ両足維持
            if(is_first_cycle) {
                r_foot.z = r_sign * dir_sign * abs_foot_dir_z;
                l_foot.z = l_sign * dir_sign * abs_foot_dir_z;
            }
        }

        return true;
    }
    ///////////////////////////////////////////////////////////////////
    //// 歩行周期管理と軸足切替 - 歩行の根幹となるタイミング制御 ////
    ///////////////////////////////////////////////////////////////////
    void counterCont(){
        float k;

        if(fwct >= fwct_end){
            // ■ 歩行周期完了時の軸足切替処理
            jikuasi ^= 1;           // 軸足切替：0↔1（右軸足↔左軸足）
            fwct = 0;               // 新周期開始

            // ■ 脚上げ高さリセット
            foot_h = 0;             // 新しい遊脚の高さを地面レベルに
            
            // ■ 左右脚の役割交換（軸足↔遊脚の位置データ入れ替え）
            k = sw_leg.y;           // 遊脚Y位置を一時保存
            sw_leg.y = sp_leg.y;    // 軸足Y位置 → 新遊脚Y位置
            sp_leg_b.y = sp_leg.y;  // 軸足Y位置のバックアップ
            sp_leg.y = k;           // 旧遊脚Y位置 → 新軸足Y位置

            k = sw_leg.x;           // 遊脚X位置を一時保存  
            sw_leg.x = sp_leg.x;    // 軸足X位置 → 新遊脚X位置
            sp_leg_b.x = sp_leg.x;  // 軸足X位置のバックアップ
            sp_leg.x = k;           // 旧遊脚X位置 → 新軸足X位置

            k = sw_leg.z;           // 遊脚Z回転角を一時保存
            sw_leg.z = sp_leg.z;    // 軸足Z回転角 → 新遊脚Z回転角
            sp_leg.z = k;           // 旧遊脚Z回転角 → 新軸足Z回転角

        }
        else{
            // ■ 歩行周期カウントアップ
            fwct += fwctUp;   // 通常は+1ずつ増加
            if(fwct > fwct_end) fwct = fwct_end;	// 上限値ガード
        }

        // 初回も通常歩行も同じ時間で安定した歩行を実現
        fwct_end = fwct_end_set;  // 常に同じ周期時間を使用
    }

    public:

    //##################
    //#### 歩行制御 ####
    //##################

/**
 * @brief uvc walk
 * @param sensor mrd_sensor data(joypad,mpu...)
 * @param s_arr meridim array
 * @return void
 * @note +Hori 20251011
 */
    void walk(mrd_sensor *(snsr),  uint16_t s_arr[], ServoParam* sv)
    {

        // 歩行モード以外は抜ける
        if(!(snsr->joy.pad_key & KB_L1)) {
            //restore_base_posture();     // パッドボタンを離したタイミングで基準姿勢に復帰 -Hori 20251018
            doing = false;
            return;
        }

        // 歩行モード   ---------------------
        switch(mode){

        //**** 0 転倒する角度に達したため停止 ****
        // リモコンのボタンを離すと解除される
        case M0:
        
            // +Hori 20240914 転倒復帰後の歩行再開対応i
            if(fallDown == true){
                Serial.println("fallDown Reset!");
                fallDown = false;
            }

            doing = false;  // 動作中フラグOFF
            break;

        //**** 開始、初期姿勢に移行 ****
        // 初期姿勢をセット
        case M710:

            // 上半身の初期姿勢セット
            TGT[ID_HEAD_Y][ID_CENTER]       = (float)sv->ixl_tgt_past[0] / 180 * M_PI;  //頭ヨー
            TGT[ID_SHOULDER_P][ID_LEG_L]    = (float)sv->ixl_tgt_past[1] / 180 * M_PI;  //肩ピッチ
            TGT[ID_SHOULDER_R][ID_LEG_L]    = (float)sv->ixl_tgt_past[2] / 180 * M_PI;  //肩ロール
            TGT[ID_SHOULDER_Y][ID_LEG_L]    = (float)sv->ixl_tgt_past[3] / 180 * M_PI;	//肩ヨー
            TGT[ID_ELBOW_P][ID_LEG_L]       = (float)sv->ixl_tgt_past[4] / 180 * M_PI;  //肘ピッチ
            TGT[ID_WAIST_Y][ID_CENTER]      = (float)sv->ixr_tgt_past[0] / 180 * M_PI;  //腰ヨー
            TGT[ID_SHOULDER_P][ID_LEG_R]    = (float)sv->ixr_tgt_past[1] / 180 * M_PI;  //肩ピッチ
            TGT[ID_SHOULDER_R][ID_LEG_R]    = (float)sv->ixr_tgt_past[2] / 180 * M_PI;  //肩ロール
            TGT[ID_SHOULDER_Y][ID_LEG_R]    = (float)sv->ixr_tgt_past[3] / 180 * M_PI;  //肩ヨー
            TGT[ID_ELBOW_P][ID_LEG_R]       = (float)sv->ixr_tgt_past[4] / 180 * M_PI;  //肘ピッチ

            // 下半身の初期姿勢セット
            for(int i=0; i<2; i++){
                TGT[ID_HIP_Y][i]        = 0.0;	    //股ヨー
                TGT[ID_HIP_R][i]        = 0.0;	    //股ロール
                TGT[ID_HIP_P][i]        = -15.0 / 180.0 * M_PI;	//股ピッチ
                TGT[ID_KNEE_P][i]       = 30.0  / 180.0 * M_PI;	//膝ピッチ
                TGT[ID_ANKLE_P][i]      = -15.0 / 180.0 * M_PI;	//足首ピッチ
                TGT[ID_ANKLE_R][i]      = 0.0;      //足首ロール
            }

            // 初期姿勢へのポーズ移行 
            if(motCt > 0){
                --motCt;

                flow_monitor(MONITOR_TYPE_MOTCT);

                if(hip_h > HIP_H_WALK)
                	hip_h -= 0.2;	

                jikuasi = ID_LEG_R;  // 軸足を右に
		        foot_control( 0, 0, hip_h,  ID_LEG_R );    // 軸足 右
		        foot_control( 0, 0, hip_h,  ID_LEG_L );    // 遊脚 左
                arm_control();

            }
            else{
                //// 角度補正用 ////
                snsr_rpy_offset.pitch   = 0;
                snsr_rpy_offset.roll	= 0;
                integ_cnt	    = 0;
                integ_a.pitch   = 0;
                integ_a.roll	= 0;
                integ_b.pitch	= 0;
                integ_b.roll	= 0;

                //// UVC積分用 ////
                sp_leg.x	= 0;
                sp_leg.y	= 0;
                sp_leg.z	= 0;
                sw_leg.x	= 0;
                sw_leg.y	= 0;
                sw_leg.z	= 0;
                sp_leg_b.x	= 0;
                sp_leg_b.y	= 0;
                sp_leg_b.z	= 0;

                land_front	= 0;
                land_back	= 0;

                walkCt	    = 0;
                fwctUp	    = 1;
                fwct        = 1;

                hip_h = HIP_H_WALK;
                sw          = 0;
                sw_sub.x    = 0;
                sw_sub.y    = 0;
                sw_sub.z    = 0;
                wt          = 0;        // 腰回転角度の初期化 +Fix
                wk          = 0;        // 腰位置の初期化 +Fix
                
                jikuasi = ID_LEG_L;  // 軸足を左に

                //// 初期姿勢 ////
                foot_control(0,0, HIP_H_WALK, ID_LEG_R);
                foot_control(0,0, HIP_H_WALK,ID_LEG_L);
                arm_control();

                // 基準姿勢保存（パッドボタン押下開始時の関節角度を保存）
                //save_base_posture();

                mode = M720;  // 状態遷移

                flow_monitor(MONITOR_TYPE_IP);
            }
            break;

    
        //**** 開始時の傾斜角補正 ****
        // IMU RPYのオフセット補正値を算出
        case M720:

            angle_adjust_imu();

            if( integ_cnt == CNT_OFFSET_MAX ){
                snsr_rpy_offset.pitch = (float)(integ_a.pitch / CNT_OFFSET_MAX);
                snsr_rpy_offset.roll  = (float)(integ_a.roll  / CNT_OFFSET_MAX);

                flow_monitor(MONITOR_TYPE_OFS);
                flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);

                mode = M730;	//状態遷移
            }
            detection_angle_falldown();			//転倒検知

            break;

        //===========================================================
        //**** M730:通常歩行状態 - CPG歩行・UVC歩行の実行 ****
        //===========================================================
        // ここから通常歩行ルーチン
        case M730:
            float k;

        /*  リアルタイムオフセット補正（現在は無効化）
            if( integ_cnt >= CNT_RETURN_LEG + CNT_RETURN_LEG ){
                integ_cnt = 0;
                if(snsr_rpy.roll >0) ++snsr_rpy_offset.roll;
                if(snsr_rpy.roll <0) --snsr_rpy_offset.roll;
                if(snsr_rpy.pitch>0) ++snsr_rpy_offset.pitch;
                if(snsr_rpy.pitch<0) --snsr_rpy_offset.pitch;
            }
            else ++integ_cnt;
        */

            // CPG歩行制御開始判定 +Hori 20231217
            if(walk_on){    // ジョイスティック指示で歩行開始
                // 歩行指示時

                if(uvc_walk_enable){        // UVC歩行機能が有効な場合
                    foot_up();			            //脚上げによる股関節角算出
                    swing_control();          	    //横振り制御
                    feet_control_2(false);		    //脚駆動 腰回転なし
                    TGT[ID_SHOULDER_R][ID_LEG_R] = UVCW_SHOULDER_ROLL; // 腕を開く
                    TGT[ID_SHOULDER_R][ID_LEG_L] = TGT[ID_SHOULDER_R][ID_LEG_R];
                    fwct = 1;                   //UVC動作開始でカウンタを最低値にセット
                    mode = M740;			    //状態遷移
                }
            }
            else{
                // 停止指示時：swing_control()を呼ばない（ベース版に合わせる）
                foot_up();			            //脚上げによる股関節角算出
                // swing_control();	    	    //横振り制御 - 停止時は不要
                feet_control_2(false);          //脚駆動 腰回転なし

                // 転倒回避動作
                if(uvc_avoid_fall_enable){

                    // ロールピッチの各傾斜角が閾値（静止時）を超えたらUVC動作へ移行
                    if(	fabs(cur_rpy.roll) > UVC_ROLL_THRES || fabs(cur_rpy.pitch) > UVC_PITCH_THRES ){

                        if(cur_rpy.roll > 0)    //右に傾いている
                            jikuasi = ID_LEG_L;     //軸足を左に
                        else
                            jikuasi = ID_LEG_R;     //軸足を右に

                        fwct = 1;                   //UVC動作開始でカウンタを最低値にセット
                        mode = M740;			    //状態遷移
                        break;
                    }
                }

            }

            
            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            break;

        //**** UVC動作開始 ****
        case M740:

            if(walk_on == false){
                // 転倒防止の基本動作
                uvc();					    //UVCメイン制御
                uvc_sub_1();			    //UVCサブ制御
                foot_up();				    //脚上げによる股関節角算出
                feet_control_2(true);	    //脚駆動 腰回転あり
                arm_control();              //腕制御
                counterCont();			    //周期カウンタの制御

                // UVC動作完了判定
                if(fwct == 0){
                    fwct = 1;       // カウンタ初期化（回復動作を長くする）
                    mode = M750;			//状態遷移
                }

            }
            else{

                // 歩行中の動作 - 全周期スムーズな段階的前進制御 +Hori 20251027
                landing_start = fwct_end * TERM_LAND_STRIDE;             // 着地開始
                landing_end = fwct_end * (1.0 - TERM_LAND_STRIDE);       // 着地終了

                feet_direction_x();   // X軸 前後方向制御
                feet_direction_y();   // Y軸 左右方向制御
                feet_direction_zw();  // Z軸 回転方向制御

                // 軸足に応じた sp_leg, sw_leg の設定
                if(jikuasi == ID_LEG_R) {   // 右足が軸足の場合
                    sp_leg.y =  r_foot.y;   // 軸足（右足）Y位置
                    sw_leg.y =  l_foot.y;   // 遊脚（左足）Y位置
                    sp_leg.z =  r_foot.z;   // 軸足（右足）Z回転角
                    sw_leg.z = -l_foot.z;   // 遊脚（左足）Z回転角
                } else {  // 左足が軸足の場合
                    sp_leg.y =  l_foot.y;   // 軸足（左足）Y位置
                    sw_leg.y =  r_foot.y;   // 遊脚（右足）Y位置
                    sp_leg.z = -l_foot.z;   // 軸足（左足）Z回転角
                    sw_leg.z =  r_foot.z;   // 遊脚（右足）Z回転角
                }
                
                // 遊脚配置：前後方向のみ
                sw_leg.x = -sp_leg.x;                                       // 前後方向は逆

                /* for debug
                Serial.print("\tr_foot.z:"); Serial.print(r_foot.z);
                Serial.print("\tl_foot.z:"); Serial.println(l_foot.z);
                */

                uvcw();					    //UVCWメイン制御
                foot_up();				    //脚上げによる股関節角算出
                feet_control_2(false);	    //脚駆動 腰回転あり
                counterCont();			    //周期カウンタの制御
                TGT[ID_SHOULDER_R][ID_LEG_R] = UVCW_SHOULDER_ROLL; // 腕を開く
                TGT[ID_SHOULDER_R][ID_LEG_L] = TGT[ID_SHOULDER_R][ID_LEG_R];

                // UVC動作完了判定
                if(fwct == 0){
                    fwct = 1;       // カウンタ初期化（回復動作を長くする）
                    mode = M740;	//状態遷移
                }
            }


            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            detection_angle_falldown();			//転倒検知
            break;

        //**** UVC後、振動減衰待ち ****
        case M750:
        
            feet_control_2(true);               //脚駆動 腰回転あり

            if(	fwct > CNT_UVC_DAMPING ){

                // 振動減衰完了
                fwct = 1;       // カウンタ初期化

                k = (float)sqrt(G_SW_XY_MID * sw_leg.x*sw_leg.x + sw_leg.y*sw_leg.y);	//移動量、前後方向は減少させる

                sw_max = (G_SWING_MID * SWING_MAX) + (G_SWING_MID * SWING_MAX) * k/STRIDE_MAX;  // 横振り最大値設定
                

                mode = M760;    //状態遷移

                // +Hori 20251025 ロールピッチの各傾斜角が閾値以上の場合はカウンタをfwct=0にして回復動作を長くする
                if( fabs(cur_rpy.roll)  > UVC_ROLL_THRES ||
                    fabs(cur_rpy.pitch) > UVC_PITCH_THRES ){
                    fwct = 1;   // カウンタ初期化（回復動作を長くする）
                    Serial.println("Extend Recovery Time!");                    
                    mode = M750;    //状態遷移（延長）
                }

                flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
                break;
            }
            else{
                // 振動減衰待ち中
                ++fwct;
            }

            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            detection_angle_falldown();			//転倒検知
            break;


        //**** 回復動作 ****
        case M760:
            // 回復動作を元の設定に近づける
            land_front = CNT_RETURN_LEG;
            fwct_end = land_front + CNT_RETURN_LEG;

            uvc_sub_2();			    //UVCサブ制御


            // 姿勢安定度チェック（閾値の50%以下なら安定と判定）+Hori 20251126
            if( fabs(cur_rpy.roll)  < UVC_ROLL_THRES * 0.5 &&
                fabs(cur_rpy.pitch) < UVC_PITCH_THRES * 0.5){
                // 姿勢安定時は両足接地を維持
                foot_h = 0;         // 足を地面に接地
                sw_sub.x = 0;       // 横振り停止
                sw_sub.y = 0;
            }else{
                foot_up();			        //脚上げによる股関節角算出
                swing_control();	    	//横振り制御
                feet_control_2(false);	    //脚駆動 腰回転なし
                arm_control();              //腕制御
            }

            counterCont();		        //周期カウンタの制御5

            if(fwct == 0){
                // 回復完了時の初期化を簡素化
                land_front = 0;
                land_back = 0;                  // land_backも明示的に初期化
                
                sp_leg.x = 0;
                sp_leg.y = 0;
                sw_leg.x = 0;
                sw_leg.y = 0;
                sw_leg.z = 0;
                sw_leg.z = 0;
                sp_leg_b.x = 0;
                sp_leg_b.y = 0;
                sp_leg_b.z = 0;
                
                fwct_end = fwct_end_set;        // 設定値を使用
                walkCt = 0;
                fwctUp = 1;
                fwct = 1;
                hip_h = HIP_H_WALK;
                sw = 0;
                sw_sub.x = 0;
                sw_sub.y = 0;
                sw_sub.z = 0;
                jikuasi = ID_LEG_L;             // 軸足を左に

                mode = M770;		            //状態遷移
            }

            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET  | MONITOR_TYPE_SW);
            detection_angle_falldown();			//転倒検知
            break;

            //**** 回復後、振動減衰待ち ****
        case M770:

            feet_control_2(false);              //脚駆動 腰回転なし

            if(	fwct > CNT_RETURN_DAMPING ){
                fwct = 1;
                mode = M730;		                //状態遷移

                // +Hori 20251025 ロールピッチの各傾斜角が閾値以上の場合はカウンタをfwct=0にして回復動作を長くする
                if( fabs(cur_rpy.roll)  > UVC_ROLL_THRES ||
                    fabs(cur_rpy.pitch) > UVC_PITCH_THRES ){
                    fwct = 1;   // カウンタ初期化（回復動作を長くする）
                    Serial.println("Extend Recovery Time!");                    
                    mode = M770;    //状態遷移（延長）
                }

                flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
                break;
            }
            else{
                ++fwct;
            }

            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            detection_angle_falldown();			//転倒検知
            break;

        case M780:
            feet_control_2(true);               //脚駆動 腰回転あり

            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            break;

        case M790:                              //テストモード適用時に必要
            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            break;

        case M791:                              //テストモード適用時に必要

            sw_leg.x = -sp_leg.x;
            sw_leg.y =  sp_leg.y;

            feet_control_2(true);               //脚駆動 腰回転あり

            flow_monitor(MONITOR_TYPE_FWCT | MONITOR_TYPE_FEET | MONITOR_TYPE_SW);
            break;

        case M700:				                //モニター

            flow_monitor(MONITOR_TYPE_DEF);
            break;

        }   // end of switch

        // 関節角度変換とジャイロフィードバック適用
        register_joint_angles();

    }   // end of class

/**
 * @brief convert value of mpu_sensor +Hori 20221001
 * @param mpu_sensor mpu_sensor data
 * @return void
 */
    void mpu_sensor_deg2rad(mpu_sensor mpu)
    {
        // deg -> radian
        snsr_acc.roll   = float(mpu.aa.x        * 1.0);
        snsr_acc.pitch  = float(mpu.aa.y        * 1.0);

        snsr_gyr.roll   = float(mpu.gyro.x      / 180 * M_PI);
        snsr_gyr.pitch  = float(mpu.gyro.y      / 180 * M_PI);

        snsr_rpy.roll       = float(mpu.rpy.roll    / 180 * M_PI);
        snsr_rpy.pitch      = float(mpu.rpy.pitch   / 180 * M_PI);

	    snsr_rpy.roll  -= snsr_rpy_offset.roll;	    //補正
    	snsr_rpy.pitch -= snsr_rpy_offset.pitch;	//補正

        cur_rpy.roll = snsr_rpy.roll;
        cur_rpy.pitch = snsr_rpy.pitch;

    }

/**
 * @brief uvc-walk initialize +Hori 20220917
 * @param none
 * @return void
 */
    void init(void)
    {
        // 積分用変数初期化
        integ_cnt = 0;
        integ_b.roll = 0;
        integ_b.pitch = 0;

        // 角度補正用変数初期化 
        snsr_rpy_offset.pitch = 0;
        snsr_rpy_offset.roll = 0;
        snsr_acc.pitch = 0;  // 初期値（MPUデータで即座に上書きされる）
        snsr_acc.roll  = 0;  // 初期値（MPUデータで即座に上書きされる）

        // 変数初期値セット
        fwct_end_set    = CNT_FWCT_END;
        sw_max_set      = SWING_MAX;
        foot_h_max_set  = FOOT_H_MAX;

        // 初期姿勢設定へ
        motCt = CNT_INIT_POSE;    // 初期姿勢カウンタ
        mode = M0;

        // UVC設定初期値
        walk_on     = false;  // 歩行開始フラグ
        fallDown    = false;  // 転倒検知フラグ
        doing       = false;  // 動作中フラグ

        foot_direction = {0,0,0}; // 足移動位置初期化
        
        // Z軸回転角初期化
        sp_leg.z = 0.0;         // 支持脚Z軸回転角初期化
        sw_leg.z = 0.0;         // 遊脚Z軸回転角初期化

        Serial.println("UVCClass. Initialized");
    }

/**
 * @brief set uvc-walk status +Hori 20220918
 * @param sensor sensor data(joypad,mpu...)
 * @return void
 */
    void set_status(mrd_sensor *(snsr), bool motion_playing)
    {
        mpu_sensor_deg2rad(snsr->mpu);

        switch(snsr->joy.pad_key){

            case KB_L1:             // WALK-MODE
            //case KB_L1 + KB_R1:     // WALK-MODE with R PUNCH

                if(motion_playing == true)
                    break;  // モーション再生中は抜ける

                doing = true;  // 動作中フラグ

                if( !(snsr->joy.pad_key_last & (KB_L1)) ){

                    // 設定初期化
                    motCt = CNT_INIT_POSE;  //初期姿勢カウンタ
                    hip_h=  HIP_H_START;	//初期姿勢高さ

                    uvc_avoid_fall_enable   = UVC_START_AVOID_FALL_CFG;   //回脚転倒回避 ON/OFF
                    uvc_walk_enable         = UVC_START_UVCW_CFG;         //UVC歩行 ON/OFF
                    mix_enable              = UVC_START_MIX_CFG;          //UVC評価でジャイロFB ON/OFF

                    // 変数初期化
                    walk_on  = false;
                    fallDown = false;

                    // 調整したパラメータをセット
                    fwct_end    = fwct_end_set; 
                    sw_max_set      = SWING_MAX_UVCW;        // UVC歩行用横振り最大距離に変更
                    foot_h_max_set  = FOOT_H_MAX_UVCW;       // UVC歩行用足上げ最大高さに変更

                    Serial.println("uvc start");

                    mode = M710;

                }
  
                                
                // 歩行開始/停止判定
                if(mode == M730 || mode == M740){

                    vector3 joy_l, joy_r;
                    float joy_len;
                    joy_l.x = (float)snsr->joy.pad_y_l / 127.0f;   // -1.0 ～ +1.0
                    joy_l.y = (float)snsr->joy.pad_x_l / 127.0f;   // -1.0 ～ +1.0
                    joy_r.x = (float)snsr->joy.pad_x_r / 127.0f;   // -1.0 ～ +1.0
                    joy_r.y = (float)snsr->joy.pad_y_r / 127.0f;   // -1.0 ～ +1.0
                    joy_len = sqrt( joy_l.x * joy_l.x + joy_l.y * joy_l.y + joy_r.x * joy_r.x + joy_r.y * joy_r.y );

                    foot_direction.x = WALK_STRIDE_FB * joy_l.x;    // 移動量をセット（float変換）
                    foot_direction.y = WALK_STRIDE_LR * joy_l.y;    // 移動量をセット（float変換）
                    foot_direction.z = WALK_TURN_LR   * joy_r.x;    // 回転量をセット（float変換）

                    // 停止中でアナログ操作ありなら歩行開始
                    // 歩行中でアナログ操作なしなら最初から
                    if( (walk_on == false) && (joy_len >= ANALOG_STICK_THRES)){

                        walk_on = true;                 

                        // 後でリアルタイムに調整する変数に初期値をセット
                        fwct_end_set    = CNT_FWCT_END_UVCW;     // UVC歩行用周期に変更
                        sw_max_set      = SWING_MAX_UVCW;        // UVC歩行用横振り最大距離に変更
                        foot_h_max_set  = FOOT_H_MAX_UVCW;       // UVC歩行用足上げ最大高さに変更

                        fwct_end    = fwct_end_set;             //一周期最大カウント数 default 48 -> 36
                        sw_max      = sw_max_set;    	        //横振り最大距離（歩き始め）defalt 24mm
                        foot_h_max  = foot_h_max_set;	        //足上げ最大高さ（歩き） default 20 mm

                    }
                    else if( (walk_on == true) && (joy_len < ANALOG_STICK_THRES) ){

                        walk_on = false;

                        //mode = M710;

                    }

                }
                
                break;

            /*
            case KB_L1 + KB_L2 + KB_TRIANGLE:           // fwct_end +
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_TRIANGLE) ){
                    fwct_end_set ++;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_X:                  // fwct_end -
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_X) ){
                    (fwct_end_set > 0) ? fwct_end_set -- : 0;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_CIRCLE:             // sw_max +
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_CIRCLE) ){
                    sw_max_set ++;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_SQUARE:             // sw_max -
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_SQUARE) ){
                    (sw_max_set > 0) ? sw_max_set -- : 0;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_TRIANGLE + KB_CIRCLE:   // foot_h_max +
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_TRIANGLE + KB_CIRCLE) ){
                    foot_h_max_set ++;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_X + KB_CIRCLE:          // foot_h_max -
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_X + KB_CIRCLE) ){
                    (foot_h_max_set > 0) ? foot_h_max_set -- : 0;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            case KB_L1 + KB_L2 + KB_START:     // reset
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_START) ){
                    fwct_end_set = 36;
                    sw_max_set = 24;
                    foot_h_max_set = 20;
                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;
            */

            case KB_L1 + KB_L2 + KB_R1 + KB_R2:     // reset
                if( snsr->joy.pad_key_last != (KB_L1 + KB_L2 + KB_R1 + KB_R2) ){
                    ahrs.yaw_origin     = mrd_wire0_setyaw(ahrs.ypr[0]);
                    ahrs.pitch_origin   = mrd_wire0_setpitch(ahrs.ypr[1]);
                    ahrs.roll_origin    = mrd_wire0_setroll(ahrs.ypr[2]);
                    Serial.println("Reset IMU's RPY origin");

                    fwct_end_set = CNT_FWCT_END;
                    sw_max_set = SWING_MAX;
                    foot_h_max_set = FOOT_H_MAX;

                    flow_monitor(MONITOR_TYPE_PRM_W);
                }
                break;                

            case KB_NONE:     // NO KEY
                mode = M0;
                doing = false; // 動作中フラグ
                break;

            default:
                doing = false; // 動作中フラグ
                break;
        }


    }
    // サーボデータをmeridim配列とサーボ構造体に書き込む +Hori 20250711
    void updateServoData(uint16_t m_arr[], ServoParam* sv) 
    {
        for (int i = 0; i < 15; i++) {
            if(sv->ixl_mount[i]){
                m_arr[(i * 2) + 20] = joint_cmd_l[i];
                m_arr[i * 2 + 21] = mrd.float2HfShort(joint_l[i]);
                sv->ixl_tgt[i] = joint_l[i];
            }
            if(sv->ixr_mount[i]){
                m_arr[(i * 2) + 50] = joint_cmd_r[i];
                m_arr[i * 2 + 51] = mrd.float2HfShort(joint_r[i]);
                sv->ixr_tgt[i] = joint_r[i];
            }
        }

        if(REG_MRD_FEET_XYZ){
            if(jikuasi == ID_LEG_R){
                // 軸足：右、遊脚：左
                foot_r.x = sp_leg.x;        // 右軸足X位置
                foot_r.y = sp_leg.y;        // 右軸足Y位置
                foot_r.z = 0;               // 右軸足Z位置
                foot_l.x = sw_leg.x;        // 左遊脚X位置
                foot_l.y = sw_leg.y;        // 左遊脚Y位置
                foot_l.z = foot_h;          // 左遊脚Z位置
            }
            else{
                // 軸足：左、遊脚：右
                foot_r.x = sw_leg.x;        // 右遊脚X位置
                foot_r.y = sw_leg.y;        // 右遊脚Y位置
                foot_r.z = foot_h;          // 右遊脚Z位置
                foot_l.x = sp_leg.x;        // 左軸足X位置
                foot_l.y = sp_leg.y;        // 左軸足Y位置
                foot_l.z = 0;               // 左軸足Z位置
            }

        }



    }

    // +Hori 20221014
    void flow_monitor(int type)
    {

        if(MONITOR_FLOW_UVC){

            Serial.print("mod: ");
            Serial.print(mode);
            Serial.print(" J: ");
            if(jikuasi == ID_LEG_R) Serial.print("R");
            else                    Serial.print("L");

            if(MONITOR_SET_RP_DEG){
                Serial.print(" RP(d):\t");
                Serial.print((cur_rpy.roll/M_PI*180));
                Serial.print("\t");
                Serial.print((cur_rpy.pitch/M_PI*180));
            }else{
                Serial.print("\tRP(r):\t");
                Serial.print(cur_rpy.roll);
                Serial.print("\t");
                Serial.print(cur_rpy.pitch);
            }

            if(type & MONITOR_TYPE_MOTCT){
                Serial.print("\tmotct: ");
                Serial.print(motCt);
            }
            if(type & MONITOR_TYPE_IP){
                Serial.print("\tip: ");
                Serial.print(integ_cnt);
            }
            if(type & MONITOR_TYPE_FWCT){
                Serial.print("\tfwct: ");
                if(fwct < 10) Serial.print("_");    // 2桁表示
                Serial.print(fwct);
            }
            if(type & MONITOR_TYPE_OFS){
                Serial.print("\troll+ofs:\t");
                Serial.print(snsr_rpy.roll*180/M_PI);
                Serial.print("\t");
                Serial.print(snsr_rpy_offset.roll*180/M_PI);
                Serial.print("\tpitch+ofs:\t");
                Serial.print(snsr_rpy.pitch*180/M_PI);
                Serial.print("\t");
                Serial.print(snsr_rpy_offset.pitch*180/M_PI);
            }
            if(type & MONITOR_TYPE_FEET){
                Serial.print("\tRxyHLxyH:\t");
                Serial.print(foot_r.x);
                Serial.print("\t");
                Serial.print(foot_r.y);
                Serial.print("\t");
                Serial.print(foot_r.z);
                Serial.print("\t"); 
                Serial.print(foot_l.x);
                Serial.print("\t");
                Serial.print(foot_l.y);
                Serial.print("\t");
                Serial.print(foot_l.z);
            }
            if(type & MONITOR_TYPE_SW){
                Serial.print("\tSwSxy:\t");
                Serial.print(sw_sub.x);
                Serial.print("\t");
                Serial.print(sw_sub.y);
            }
            if(type & MONITOR_TYPE_HIP){
                Serial.print("\tHIPh:\t");
                Serial.print(hip_h);
            }
            if(type & MONITOR_TYPE_PRM_W){
                Serial.print("\tfwct_end\t");
                Serial.print(fwct_end_set);
                Serial.print("\tsw_max\t");
                Serial.print(sw_max_set);
                Serial.print("\tfoot_h_max\t");
                Serial.print(foot_h_max_set);
            }
            Serial.println();
        }

    }

    ////////////////////////////////////////////////////////////////////////////////
    //	基準姿勢保存・復帰関数
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief 基準姿勢へ遷移時に現在の関節角度を基準姿勢として保存
     * @note パッドボタン押下開始時の姿勢を記録し、復帰時の基準姿勢とする
     */
    void save_base_posture() {

        // 現在の関節角度を基準姿勢として保存
        for(int i = 0; i < 15; i++) {
            base_l[i] = joint_l[i];
            base_r[i] = joint_r[i];
        }
        base_saved = true;
        
        Serial.println("Base posture saved");
    }
    
    /**
     * @brief パッドボタンを離した時に保存済み基準姿勢に復帰
     * @note UVC制御終了時に初期姿勢へ安全に戻る
     */
    void restore_base_posture() {

        if(base_saved) {
            // 保存済み基準姿勢に復帰
            for(int i = 0; i < 15; i++) {
                joint_l[i] = base_l[i];
                joint_r[i] = base_r[i];
            }

            Serial.println("Base posture restored");
        }
    }

    /**
     * @brief TGT配列から関節角度配列への変換とジャイロフィードバック適用
     * @note ラジアン→度変換、左右関節への角度設定、ジャイロによる姿勢補正
     */
    void register_joint_angles() {
    
        // トルクオン +Hori 20220917
        for(int i=0;i<15;i++){
            joint_cmd_l[i] = 1;
            joint_cmd_r[i] = 1;
        }

        // +UVC 20220810 Set Joint Value
        joint_l[0]  =  (float)(TGT[ID_HEAD_Y][ID_CENTER]    *  180 / M_PI);     //頭ヨー
        joint_l[1]  =  (float)(TGT[ID_SHOULDER_P][ID_LEG_L] *  180 / M_PI);     //股ピッチ
        joint_l[2]  =  (float)(TGT[ID_SHOULDER_R][ID_LEG_L] *  180 / M_PI);     //肩ロール 
        joint_l[3]  =  (float)(TGT[ID_SHOULDER_Y][ID_LEG_L] *  180 / M_PI);     //肩ヨー 
        joint_l[4]  =  (float)(TGT[ID_ELBOW_P][ID_LEG_L]    *  180 / M_PI);     //肘ピッチ 
        joint_l[5]  =  (float)(TGT[ID_HIP_Y][ID_LEG_L]      *  180 / M_PI);     //股ヨー 
        joint_l[6]  =  (float)(TGT[ID_HIP_R][ID_LEG_L]      *  180 / M_PI);     //股ロール 
        joint_l[7]  =  (float)(TGT[ID_HIP_P][ID_LEG_L]      * -180 / M_PI);     //股ピッチ 
        joint_l[8]  =  (float)(TGT[ID_KNEE_P][ID_LEG_L]     *  180 / M_PI);     //膝ピッチ 
        joint_l[9]  =  (float)(TGT[ID_ANKLE_P][ID_LEG_L]    * -180 / M_PI);     //足首ピッチ 
        joint_l[10] =  (float)(TGT[ID_ANKLE_R][ID_LEG_L]    *  180 / M_PI);     //足首ロール

        joint_r[0]  =  (float)(TGT[ID_WAIST_Y][ID_CENTER]   *  180 / M_PI);    //腰ヨー
        joint_r[1]  =  (float)(TGT[ID_SHOULDER_P][ID_LEG_R] *  180 / M_PI);    //股ピッチ 
        joint_r[2]  =  (float)(TGT[ID_SHOULDER_R][ID_LEG_R] *  180 / M_PI);    //肩ロール 
        joint_r[3]  =  (float)(TGT[ID_SHOULDER_Y][ID_LEG_R] *  180 / M_PI);    //肩ヨー 
        joint_r[4]  =  (float)(TGT[ID_ELBOW_P][ID_LEG_R]    *  180 / M_PI);    //肘ピッチ 
        joint_r[5]  =  (float)(TGT[ID_HIP_Y][ID_LEG_R]      *  180 / M_PI);    //股ヨー 
        joint_r[6]  =  (float)(TGT[ID_HIP_R][ID_LEG_R]      *  180 / M_PI);    //股ロール 
        joint_r[7]  =  (float)(TGT[ID_HIP_P][ID_LEG_R]      * -180 / M_PI);    //股ピッチ 
        joint_r[8]  =  (float)(TGT[ID_KNEE_P][ID_LEG_R]     *  180 / M_PI);    //膝ピッチ 
        joint_r[9]  =  (float)(TGT[ID_ANKLE_P][ID_LEG_R]    * -180 / M_PI);    //足首ピッチ 
        joint_r[10] =  (float)(TGT[ID_ANKLE_R][ID_LEG_R]    *  180 / M_PI);    //足首ロール 

        if(mix_enable == true){               //UVC評価でジャイロON/OFF
            for (int i = 0; i < 15; i++){       //  deg * (hfgain*0.01)
                joint_l[i]  += snsr_gyr.roll  * 180 / M_PI * (float)(mv_mix_l[MIX_GYRO_ROLL][i])  *  0.01;
                joint_l[i]  += snsr_gyr.pitch * 180 / M_PI * (float)(mv_mix_l[MIX_GYRO_PITCH][i]) *  0.01;
                joint_r[i]  += snsr_gyr.roll  * 180 / M_PI * (float)(mv_mix_r[MIX_GYRO_ROLL][i])  *  0.01;
                joint_r[i]  += snsr_gyr.pitch * 180 / M_PI * (float)(mv_mix_r[MIX_GYRO_PITCH][i]) *  0.01;
            }
        }
    }

};  // end-of-uvcclass
