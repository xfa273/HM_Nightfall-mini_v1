/**
 * @file shortest_run_params.h
 * @brief モード3～7の最短走行(case4～7)用パラメータ設定ヘッダ
 */

#ifndef SHORTEST_RUN_PARAMS_H
#define SHORTEST_RUN_PARAMS_H

#include <stdint.h>

/**
 * @brief 最短走行パラメータ構造体
 */
typedef struct {
    float distance;      ///< 距離 (mm)
    float velocity;      ///< 速度 (m/s)
    float acceleration;  ///< 加速度 (m/s^2)
    // 直線
    float acceleration_straight;      ///< 直線加速度 (m/s^2)
    float acceleration_straight_dash; ///< 二段階直線加速度 (m/s^2)
    float velocity_straight;          ///< 直線速度 (mm/s)
    // ターン
    float velocity_turn90;            ///< 90°ターン速度 (mm/s)
    float alpha_turn90;               ///< 90°ターンハンドリングパラメータ
    float acceleration_turn;          ///< ターン加速度 (m/s^2)
    float dist_offset_in;             ///< 内側オフセット (mm)
    float dist_offset_out;            ///< 外側オフセット (mm)
    float val_offset_in;              ///< 内部位置補正値
    float angle_turn_90;              ///< 90°ターン角度設定 (度)
    // 90°大回りターン
    float velocity_l_turn_90;         ///< 大回り90°速度 (mm/s)
    float alpha_l_turn_90;            ///< 大回り90°ハンドリングパラメータ
    float angle_l_turn_90;            ///< 大回り90°ターン角度 (度)
    float dist_l_turn_in_90;          ///< 大回り90°入り距離 (mm)
    float dist_l_turn_out_90;         ///< 大回り90°オフセット (mm)
    // 180°大回りターン
    float velocity_l_turn_180;        ///< 大回り180°速度 (mm/s)
    float alpha_l_turn_180;           ///< 大回り180°ハンドリングパラメータ
    float angle_l_turn_180;           ///< 大回り180°ターン角度 (度)
    float dist_l_turn_in_180;         ///< 大回り180°入り距離 (mm)
    float dist_l_turn_out_180;        ///< 大回り180°オフセット (mm)
} ShortestRunParam_t;

// モード2～7それぞれ4つのケース(case4～7)用パラメータ配列
extern const ShortestRunParam_t shortestRunParamsMode2[4];
extern const ShortestRunParam_t shortestRunParamsMode3[4];
extern const ShortestRunParam_t shortestRunParamsMode4[4];
extern const ShortestRunParam_t shortestRunParamsMode5[4];
extern const ShortestRunParam_t shortestRunParamsMode6[4];
extern const ShortestRunParam_t shortestRunParamsMode7[4];

#endif // SHORTEST_RUN_PARAMS_H
