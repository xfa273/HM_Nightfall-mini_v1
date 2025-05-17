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
#define D_TIRE 23.8F            // タイヤ直径[mm]
#define DIST_HALF_SEC 90        // 迷路の半区間距離[mm]
#define DIST_D_HALF_SEC 127.279 // 斜めの半区間距離[mm]
#define DIST_FIRST_SEC 28       // 最初の区画の距離[mm]
#define DIST_SET_POSITION 30    // 壁当て後の前進距離[mm]

#define ALPHA_ROTATE_90 1400    // 超信地旋回の角加速度[deg/sec^2]
#define ANGLE_ROTATE_90_R 90.0F // 超信地旋回の角度[deg]
#define ANGLE_ROTATE_90_L 90.0F // 超信地旋回の角度[deg]

#define DIFF_SETPOSITION 500 // スラロームを位置合わせに変更する制御量

/*PIDパラメータ*/
#define KP_DISTANCE 14.0F // 並進位置制御のP項
#define KI_DISTANCE 0.01F // 並進位置制御のI項
#define KD_DISTANCE 14.0F // 並進位置制御のD項

#define KP_VELOCITY 6.0F   // 並進速度制御のP項
#define KI_VEROCITY 0.016F // 並進速度制御のI項
#define KD_VEROCITY 9.0F   // 並進速度制御のD項

#define KP_ANGLE 0.0F // 角度制御のP項
#define KI_ANGLE 0.0F // 角度制御のI項
#define KD_ANGLE 0.0F // 角度制御のD項

#define KP_OMEGA 1.5F   // 角速度制御のP項 0.45F
#define KI_OMEGA 0.02F  // 角速度制御のI項 0.015F
#define KD_OMEGA 0.0F   // 角速度制御のD項 0.06F
#define FF_OMEGA 0.006F // 角速度制御のFF項 0.012F

#define KP_IMU 1.0F // IMUの角速度の補正係数

#define FAIL_COUNT_LR 50 // 左右差フェイルセーフ発動までのカウント数[ms]
#define FAIL_LR_ERROR 10000 // 左右差フェイルセーフ発動のモータ出力左右差
#define FAIL_COUNT_ACC 20 // 衝突フェイルセーフ発動までのカウント数[ms]
#define FAIL_ACC 17000 // 衝突フェイルセーフ発動の加速度

/*動作方向関連*/

#define DIR_FWD_L GPIO_PIN_RESET  // CW/CCWで前に進む出力（左）
#define DIR_BACK_L GPIO_PIN_SET   // CW/CCWで後ろに進む出力（左）
#define DIR_FWD_R GPIO_PIN_SET    // CW/CCWで前に進む出力（右）
#define DIR_BACK_R GPIO_PIN_RESET // CW/CCWで後ろに進む出力（右）

/*------------------------------------------------------------
    センサ系
------------------------------------------------------------*/
/*壁判断閾値*/
#define WALL_BASE_FR 280 // 前壁右センサ    //700
#define WALL_BASE_FL 280 // 前壁左センサ    //700
#define WALL_BASE_R 370  // 右壁センサ  //800
#define WALL_BASE_L 370  // 左壁センサ  //800
#define WALL_DIFF_THR 22 // 壁センサ値の変化量のしきい値
#define K_SENSOR 1.00F   // センサの補正値 0.94F

#define WALL_CTRL_BASE_L 605 // 壁制御の基準値（左） 668
#define WALL_CTRL_BASE_R 859 // 壁制御の基準値（右） 1101

/*制御閾値*/
#define CTRL_BASE_L 1       // 左制御閾値
#define CTRL_BASE_R 1       // 右制御閾値
#define WALL_CTRL_MAX 0.002 // 制御量上限値
#define KP_DEFAULT 0.1F     // 比例制御係数
#define KP_TURN_AP 0.3F // スラロームのオフセット区間用比例制御係数

//----赤外線（赤色）LED発光待機時間（単位はマイクロ秒）
#define IR_WAIT_US 20

/*------------------------------------------------------------
    探索系
------------------------------------------------------------*/
//----ゴール座標----
#define GOAL_X 1 // 7
#define GOAL_Y 0 // 7
#define MAZE_SIZE 4
#define START_X 0
#define START_Y 0

#endif /* INC_PARAMS_H_ */
