/**
 * @file shortest_run_params.h
 * @brief 最短走行パラメータ（モード共通・ケース個別）定義
 */

#ifndef SHORTEST_RUN_PARAMS_H
#define SHORTEST_RUN_PARAMS_H

#include <stdint.h>

/**
 * @brief モード共通（ターン＋オフセット＋ファン）パラメータ
 */
typedef struct {
    // ターン（小回り）
    float velocity_turn90;            ///< 90°ターン速度 (mm/s)
    float alpha_turn90;               ///< 90°ターンハンドリングパラメータ
    float acceleration_turn;          ///< ターン加速度 (m/s^2)
    float dist_offset_in;             ///< 内側オフセット (mm)
    float dist_offset_out;            ///< 外側オフセット (mm)
    float val_offset_in;              ///< 内部位置補正値
    float angle_turn_90;              ///< 90°ターン角度設定 (度)
    // 大回りターン
    float velocity_l_turn_90;         ///< 大回り90°速度 (mm/s)
    float alpha_l_turn_90;            ///< 大回り90°ハンドリングパラメータ
    float angle_l_turn_90;            ///< 大回り90°ターン角度 (度)
    float dist_l_turn_in_90;          ///< 大回り90°入り距離 (mm)
    float dist_l_turn_out_90;         ///< 大回り90°オフセット (mm)
    float velocity_l_turn_180;        ///< 大回り180°速度 (mm/s)
    float alpha_l_turn_180;           ///< 大回り180°ハンドリングパラメータ
    float angle_l_turn_180;           ///< 大回り180°ターン角度 (度)
    float dist_l_turn_in_180;         ///< 大回り180°入り距離 (mm)
    float dist_l_turn_out_180;        ///< 大回り180°オフセット (mm)
    // ファン
    uint16_t fan_power;               ///< ファン出力 (0-1000)
} ShortestRunModeParams_t;

/**
 * @brief ケース個別（直線＋壁制御＋経路重み）パラメータ
 */
typedef struct {
    // 直線
    float acceleration_straight;      ///< 直線加速度 (mm/s^2)
    float acceleration_straight_dash; ///< 二段階直線加速度 (mm/s^2)
    float velocity_straight;          ///< 直線速度 (mm/s)
    // 壁制御
    float kp_wall;                    ///< 壁制御比例ゲイン
    // 経路重み
    int   straight_weight;            ///< 経路導出: 直線の優先度
    int   diagonal_weight;            ///< 経路導出: 斜めの優先度
} ShortestRunCaseParams_t;

// モード共通パラメータ
extern const ShortestRunModeParams_t shortestRunModeParams2;
extern const ShortestRunModeParams_t shortestRunModeParams3;
extern const ShortestRunModeParams_t shortestRunModeParams4;
extern const ShortestRunModeParams_t shortestRunModeParams5;
extern const ShortestRunModeParams_t shortestRunModeParams6;
extern const ShortestRunModeParams_t shortestRunModeParams7;

// ケース個別パラメータ（case4..7の順に4要素）
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode2[4];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode3[4];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode4[4];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode5[4];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode6[4];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode7[4];

#endif // SHORTEST_RUN_PARAMS_H
