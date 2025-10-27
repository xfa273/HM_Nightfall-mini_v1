/*
 * params.h
 *
 *  Created on: Feb 28, 2022
 *      Author: yuho-
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

/*============================================================
    各種定数（パラメータ）設定
============================================================*/
/*------------------------------------------------------------
    走行系
------------------------------------------------------------*/
/*走行パラメータ*/
#define D_TIRE            13.73F  // タイヤ直径[mm] 13.75F
#define DIST_HALF_SEC     45     // 迷路の半区間距離[mm]
#define DIST_D_HALF_SEC   67.279 // 斜めの半区間距離[mm]
#define DIST_FIRST_SEC    13     // 最初の区画の距離[mm]
#define DIST_SET_POSITION 13     // 壁当て後の前進距離[mm]

#define ALPHA_ROTATE_90   3000  // 超信地旋回の角加速度[deg/sec^2]
#define ANGLE_ROTATE_90_R 89.0F // 超信地旋回の角度[deg]
#define ANGLE_ROTATE_90_L 89.0F // 超信地旋回の角度[deg]

#define DIFF_SETPOSITION 1500 // スラロームを位置合わせに変更する制御量

/*PIDパラメータ*/
#define KP_DISTANCE 28.0F // 並進位置制御のP項  28.0F 30.0
#define KI_DISTANCE 0.05 // 並進位置制御のI項  0.01F 0.04
#define KD_DISTANCE 7.0F // 並進位置制御のD項  28.0F 150.0

#define KP_VELOCITY 1.9F // 並進速度制御のP項  50.0F 10.0
#define KI_VELOCITY 0.0F// 並進速度制御のI項  0.05F 0.04
#define KD_VELOCITY 45.0F // 並進速度制御のD項  60.0F 100.0

#define KP_ANGLE 0.0F // 角度制御のP項
#define KI_ANGLE 0.0F // 角度制御のI項
#define KD_ANGLE 0.0F // 角度制御のD項

#define KP_OMEGA 1.4F  // 角速度制御のP項 1.84F
#define KI_OMEGA 0.02F // 角速度制御のI項 0.075F
#define KD_OMEGA 0.2F  // 角速度制御のD項 0.16F
#define FF_OMEGA 0.0F // 角速度制御のFF項 0.043F

#define KP_IMU 1.0F // IMUの角速度の補正係数

#define FAIL_COUNT_LR  50    // 左右差フェイルセーフ発動までのカウント数[ms]
#define FAIL_LR_ERROR  10000 // 左右差フェイルセーフ発動のモータ出力左右差
#define FAIL_COUNT_ACC 20    // 衝突フェイルセーフ発動までのカウント数[ms]
#define FAIL_ACC       17000 // 衝突フェイルセーフ発動の加速度

/*動作方向関連*/

#define DIR_FWD_L  GPIO_PIN_RESET // CW/CCWで前に進む出力（左）
#define DIR_BACK_L GPIO_PIN_SET   // CW/CCWで後ろに進む出力（左）
#define DIR_FWD_R  GPIO_PIN_SET   // CW/CCWで前に進む出力（右）
#define DIR_BACK_R GPIO_PIN_RESET // CW/CCWで後ろに進む出力（右）
#define DIR_ENC_R  -1             // エンコーダ方向（右）
#define DIR_ENC_L  -1             // エンコーダ方向（左）

/*------------------------------------------------------------
    センサ系
------------------------------------------------------------*/
/*壁判断閾値*/
#define WALL_BASE_FR  360   // 前壁右センサ    //700
#define WALL_BASE_FL  360   // 前壁左センサ    //700
#define WALL_BASE_R   410   // 右壁センサ  //800
#define WALL_BASE_L   410   // 左壁センサ  //800
#define WALL_DIFF_THR 22   // 壁センサ値の変化量のしきい値
#define K_SENSOR      1.00F // センサの補正値 0.94F

// 壁切れ判定専用しきい値（高速走行向けに独立調整可能）
// 既定値は探索用と同一。必要に応じて実機に合わせて変更してください。
#ifndef WALL_END_THR_R
#define WALL_END_THR_R  250
#endif
#ifndef WALL_END_THR_L
#define WALL_END_THR_L  250
#endif

// 壁切れバッファ距離（ターン前に等速で走る距離）[mm]
// 例: 20mm。小回り時は半区間(DIST_HALF_SEC)を追加で短縮・追従する実装のため、
// この値は「基本バッファ」として機能します。
#ifndef WALL_END_BUFFER_MM
#define WALL_END_BUFFER_MM  40.0F
#endif

// 壁切れ未検知時の最大延長距離（本来の距離に追加して等速で探す上限）[mm]
// 例: 20mm。未検知でも暴走しないよう上限を設けるための値です。
#ifndef WALL_END_EXTEND_MAX_MM
#define WALL_END_EXTEND_MAX_MM  20.0F
#endif

#define WALL_CTRL_BASE_L 1941 // 壁制御の基準値（左） 2135
#define WALL_CTRL_BASE_R 1989 // 壁制御の基準値（右） 2100
// 小鷺田寮: L1941 R1989
// 九州: L1861 R2060

// バッテリー電圧の警告しきい値（ADCカウント）
// 例: 3000。割り込み内/起動時チェックで共通利用。
#ifndef BAT_WARN_ADC_THR
#define BAT_WARN_ADC_THR 2150
#endif

/*制御閾値*/
#define CTRL_BASE_L   1     // 左制御閾値
#define CTRL_BASE_R   1     // 右制御閾値
#define WALL_CTRL_MAX 0.002 // 制御量上限値
#define KP_DEFAULT    0.1F  // 比例制御係数
#define KP_TURN_AP    0.3F  // スラロームのオフセット区間用比例制御係数

//----赤外線（赤色）LED発光待機時間（単位はマイクロ秒）
#define IR_WAIT_US 30

// 探索中の横壁ズレ検出しきい値（wall_PIDで算出するlatest_wall_error[ADcount]の絶対値）
#define WALL_ALIGN_ERR_THR  700

/* 前壁センサを用いた中央合わせ（非接触）用パラメータ */
// 区画中央における前壁センサの目標値（実機で調整）
#define F_ALIGN_TARGET_FR    3750
#define F_ALIGN_TARGET_FL    3790
// 小鷺田寮: FR3750 FL3790
// 九州: FR3587 FL3587

// アライン実行条件（前壁が十分に見えているか判定する閾値）
#define F_ALIGN_DETECT_THR   500

// 閉ループ制御ゲイン（実機調整用）
#define MATCH_POS_KP_TRANS   0.3F   // [mm/s] / [ADcount]
#define MATCH_POS_KP_ROT     0.2F   // [deg/s] / [ADcount]

// 飽和・許容値・タイムアウト
#define MATCH_POS_VEL_MAX     200.0F   // [mm/s]
#define MATCH_POS_OMEGA_MAX   300.0F   // [deg/s]
#define MATCH_POS_TOL         100       // [ADcount]
#define MATCH_POS_TOL_ANGLE   40       // [ADcount]
#define MATCH_POS_TIMEOUT_MS  20     // [ms]
// 収束判定：FR/FL が目標±MATCH_POS_TOL 内に連続して入る必要回数（2ms/loop前提）
#define MATCH_POS_STABLE_COUNT 100      // [loop] ≒ 400ms

/*------------------------------------------------------------
    探索系
------------------------------------------------------------*/
//----ゴール座標----
#define GOAL_X    7 // 7
#define GOAL_Y    7 // 7
#define MAZE_SIZE 16
#define START_X   0
#define START_Y   0

// 複数ゴール設定（最大3x3=9個）。
// 1セル/2x2で使用する場合は未使用分を (0,0) とし、無視します。
// 既定では GOAL1 に従来の GOAL_X/GOAL_Y を入れ、その他は (0,0)。
#ifndef GOAL1_X
#define GOAL1_X GOAL_X
#define GOAL1_Y GOAL_Y
#endif

#ifndef GOAL2_X
#define GOAL2_X 7
#define GOAL2_Y 8
#endif

#ifndef GOAL3_X
#define GOAL3_X 8
#define GOAL3_Y 7
#endif

#ifndef GOAL4_X
#define GOAL4_X 8
#define GOAL4_Y 8
#endif

#ifndef GOAL5_X
#define GOAL5_X 0
#define GOAL5_Y 0
#endif

#ifndef GOAL6_X
#define GOAL6_X 0
#define GOAL6_Y 0
#endif

#ifndef GOAL7_X
#define GOAL7_X 0
#define GOAL7_Y 0
#endif

#ifndef GOAL8_X
#define GOAL8_X 0
#define GOAL8_Y 0
#endif

#ifndef GOAL9_X
#define GOAL9_X 0
#define GOAL9_Y 0
#endif

#endif /* INC_PARAMS_H_ */
