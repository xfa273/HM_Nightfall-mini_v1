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
#define D_TIRE            13.315F  // タイヤ直径[mm]
#define DIST_HALF_SEC     45     // 迷路の半区間距離[mm]
#define DIST_D_HALF_SEC   67.279 // 斜めの半区間距離[mm]
#define DIST_FIRST_SEC    14     // 最初の区画の距離[mm]
#define DIST_SET_POSITION 15     // 壁当て後の前進距離[mm]

#define ALPHA_ROTATE_90   3000  // 超信地旋回の角加速度[deg/sec^2]
#define ANGLE_ROTATE_90_R 89.5F // 超信地旋回の角度[deg]
#define ANGLE_ROTATE_90_L 89.5F // 超信地旋回の角度[deg]

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
#define WALL_BASE_FR  400   // 前壁右センサ    //700
#define WALL_BASE_FL  400   // 前壁左センサ    //700
#define WALL_BASE_R   400   // 右壁センサ  //800
#define WALL_BASE_L   400   // 左壁センサ  //800
#define WALL_DIFF_THR 22   // 壁センサ値の変化量のしきい値
#define K_SENSOR      1.00F // センサの補正値 0.94F

#define WALL_CTRL_BASE_L 1750 // 壁制御の基準値（左） 668
#define WALL_CTRL_BASE_R 2110 // 壁制御の基準値（右） 1101

/*制御閾値*/
#define CTRL_BASE_L   1     // 左制御閾値
#define CTRL_BASE_R   1     // 右制御閾値
#define WALL_CTRL_MAX 0.002 // 制御量上限値
#define KP_DEFAULT    0.1F  // 比例制御係数
#define KP_TURN_AP    0.3F  // スラロームのオフセット区間用比例制御係数

//----赤外線（赤色）LED発光待機時間（単位はマイクロ秒）
#define IR_WAIT_US 30

// 探索中の横壁ズレ検出しきい値（wall_PIDで算出するlatest_wall_error[ADcount]の絶対値）
#define WALL_ALIGN_ERR_THR  500

/* 前壁センサを用いた中央合わせ（非接触）用パラメータ */
// 区画中央における前壁センサの目標値（実機で調整）
#define F_ALIGN_TARGET_FR    3600
#define F_ALIGN_TARGET_FL    3600

// アライン実行条件（前壁が十分に見えているか判定する閾値）
#define F_ALIGN_DETECT_THR   400

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
#define GOAL_X    1 // 7
#define GOAL_Y    0 // 7
#define MAZE_SIZE 16
#define START_X   0
#define START_Y   0

#endif /* INC_PARAMS_H_ */
