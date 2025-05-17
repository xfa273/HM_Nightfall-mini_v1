/**
 * @file turn_time_calculator.h
 * @brief ターン走行の所要時間を計算する関数のヘッダーファイル
 */

#ifndef TURN_TIME_CALCULATOR_H
#define TURN_TIME_CALCULATOR_H

#include <math.h>
#include <stdint.h>
#include "shortest_run_params.h"

// path配列で表される動作の枠
// 上位1桁が動作の種類、下位2桁が回数
#define ACTION_TYPE_STRAIGHT  200   // 直進 (例: 201=1マス直進)
#define ACTION_TYPE_SMALL_R   300   // 右小回りターン (例: 301=90度右小回り)
#define ACTION_TYPE_SMALL_L   400   // 左小回りターン (例: 401=90度左小回り)
#define ACTION_TYPE_LARGE_R   500   // 右大回りターン (例: 501=90度右大回り、502=180度右大回り)
#define ACTION_TYPE_LARGE_L   600   // 左大回りターン (例: 601=90度左大回り、602=180度左大回り)

// 45度ターン系
#define ACTION_TURN_R45_IN    701   // 45度ターン右入り
#define ACTION_TURN_L45_IN    702   // 45度ターン左入り
#define ACTION_TURN_R45_OUT   703   // 45度ターン右出
#define ACTION_TURN_L45_OUT   704   // 45度ターン左出

/**
 * @brief ターン走行の所要時間を計算する
 * @param turning_angle_deg 旋回角度 [deg]
 * @param translational_velocity 並進速度 [mm/s]
 * @param angular_acceleration_deg 角加速度 [deg/s²]
 * @param front_offset_distance 前オフセット距離 [mm]
 * @param rear_offset_distance 後オフセット距離 [mm]
 * @return ターン走行の所要時間 [s]
 */
float calculate_turn_time(float turning_angle_deg, float translational_velocity, 
                         float angular_acceleration_deg, float front_offset_distance, 
                         float rear_offset_distance);

/**
 * @brief shortest_run_paramsからパラメータを取得してターン所要時間を計算
 * @param params 走行パラメータ構造体
 * @return 各種ターンの所要時間を計算して表示
 */
void print_turn_times(const ShortestRunParam_t *params);

/**
 * @brief 外部配列pathから走行全体の所要時間を計算し表示する
 * @param path 走行動作を格納したpath配列
 * @param path_size path配列のサイズ
 * @param params 走行パラメータ構造体
 * @return 走行全体の所要時間 [s]
 */
float calculate_total_run_time(const uint16_t *path, int path_size, const ShortestRunParam_t *params);

/**
 * @brief path配列の内容を解析して各動作の所要時間を表示する
 * @param path 走行動作を格納したpath配列
 * @param path_size path配列のサイズ
 * @param params 走行パラメータ構造体
 */
void print_path_times(const uint16_t *path, int path_size, const ShortestRunParam_t *params);

#endif /* TURN_TIME_CALCULATOR_H */
