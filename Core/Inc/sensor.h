/*
 * sensor.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdbool.h>

/*============================================================
    各種定数・変数宣言
============================================================*/

// IMUモデル種別
#define IMU_MODEL_NONE      0
#define IMU_MODEL_ICM20689  1
#define IMU_MODEL_ISM330    2

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合

/*グローバル変数の定義*/
//----その他----
uint8_t tp;                                         // タスクポインタ
volatile uint16_t ad_r, ad_r_off, ad_fr, ad_fr_off, ad_fl, ad_fl_off, ad_l, ad_l_off, ad_r_raw, ad_l_raw, ad_fr_raw, ad_fl_raw, ad_bat; // A-D値格納
volatile uint16_t wall_offset_r, wall_offset_l, wall_offset_fr, wall_offset_fl; // LED ON時の壁なし基準値
volatile uint16_t base_l, base_r, base_f;           // 基準値を格納
volatile int16_t dif_l, dif_r;                      // AD値と基準との差
float imu_offset_z;

float omega_x_raw, omega_y_raw, omega_z_raw;
float accel_x_raw, accel_y_raw, accel_z_raw;
float omega_x_true, omega_y_true, omega_z_true;
float accel_x_true, accel_y_true, accel_z_true;
float omega_x_offset, omega_y_offset, omega_z_offset;
float accel_x_offset, accel_y_offset, accel_z_offset;

uint8_t set_flag;
uint16_t gyro_calib_cnt;
uint8_t gyro_calib_flag;

// 現在使用しているIMUモデル
uint8_t imu_model;

#else // main.c以外からこのファイルが呼ばれている場合

extern uint8_t tp;
extern volatile uint16_t ad_r, ad_r_off, ad_fr, ad_fr_off, ad_fl, ad_fl_off, ad_l, ad_l_off, ad_r_raw, ad_l_raw, ad_fr_raw, ad_fl_raw, ad_bat; // A-D値格納
extern volatile uint16_t wall_offset_r, wall_offset_l, wall_offset_fr, wall_offset_fl; // LED ON時の壁なし基準値
extern volatile uint16_t base_l, base_r, base_f;
extern volatile int16_t dif_l, dif_r;
extern float imu_offset_z;

extern float omega_x_raw, omega_y_raw, omega_z_raw;
extern float accel_x_raw, accel_y_raw, accel_z_raw;
extern float omega_x_true, omega_y_true, omega_z_true;
extern float accel_x_true, accel_y_true, accel_z_true;
extern float omega_x_offset, omega_y_offset, omega_z_offset;
extern float accel_x_offset, accel_y_offset, accel_z_offset;

extern uint8_t set_flag;
extern uint16_t gyro_calib_cnt;
extern uint8_t gyro_calib_flag;

// 現在使用しているIMUモデル
extern uint8_t imu_model;

#endif

/*============================================================
    関数プロトタイプ宣言
============================================================*/

void sensor_init(void);
int get_adc_value(ADC_HandleTypeDef *, uint32_t);
int get_sensor_value_r(void);
int get_sensor_value_fr(void);
int get_sensor_value_fl(void);
int get_sensor_value_l(void);
int get_battery_value(void);

uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
void ICM20689_Init(void);
float ICM20689_GYRO_READ(uint8_t);
float ICM20689_ACCEL_READ(uint8_t);
void ICM20689_DataUpdate(void);

// ISM330DHCX
void ISM330_Init(void);
void ISM330_DataUpdate(void);

// 自動検出および共通アップデート
void IMU_Init_Auto(void);
void IMU_DataUpdate(void);
// WHO_AM_Iのデバッグプローブ
void IMU_ProbeWHOAMI_Debug(void);
void get_sensor_offsets(void);
void IMU_GetOffset(void);
uint8_t get_base();   // センサ基準値を取得
void get_wall_info(); // 壁情報を読む
void indicate_sensor();
void wall_end();
// 壁切れ検知（横壁の立ち下がりエッジ検出）
void detect_wall_end(void);

// フラッシュ保存/読込API（迷路用領域とは別セクタに保存）
// センサ基準値・オフセットなどを保存/読込する
// 戻り値: 読込はtrueで有効データ、falseで未初期化または破損
bool sensor_params_load_from_flash(void);
HAL_StatusTypeDef sensor_params_save_to_flash(void);
HAL_StatusTypeDef sensor_recalibrate_and_save(void);

#endif /* INC_SENSOR_H_ */
