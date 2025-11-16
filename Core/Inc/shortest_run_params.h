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
    float angle_turn_90;              ///< 90°ターン角度設定 (度)
    // 壁切れ後の追従距離
    float dist_wall_end;              ///< 壁切れ検知後にターン開始までに直進する距離 (mm)
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
    // 斜め45°（入り/出）
    float velocity_turn45in;          ///< 45°入り 速度 (mm/s)
    float alpha_turn45in;             ///< 45°入り ハンドリング
    float angle_turn45in;             ///< 45°入り 角度 (度)
    float dist_turn45in_in;           ///< 45°入り 前直進距離 (mm)
    float dist_turn45in_out;          ///< 45°入り 出オフセット距離 (mm)
    float velocity_turn45out;         ///< 45°出 速度 (mm/s)
    float alpha_turn45out;            ///< 45°出 ハンドリング
    float angle_turn45out;            ///< 45°出 角度 (度)
    float dist_turn45out_in;          ///< 45°出 入りオフセット (mm)
    float dist_turn45out_out;         ///< 45°出 出オフセット (mm)
    // 斜めV90°
    float velocity_turnV90;           ///< V90° 速度 (mm/s)
    float alpha_turnV90;              ///< V90° ハンドリング
    float angle_turnV90;              ///< V90° 角度 (度)
    float dist_turnV90_in;            ///< V90° 入りオフセット (mm)
    float dist_turnV90_out;           ///< V90° 出オフセット (mm)
    // 斜め135°（入り/出）
    float velocity_turn135in;         ///< 135°入り 速度 (mm/s)
    float alpha_turn135in;            ///< 135°入り ハンドリング
    float angle_turn135in;            ///< 135°入り 角度 (度)
    float dist_turn135in_in;          ///< 135°入り 入りオフセット (mm)
    float dist_turn135in_out;         ///< 135°入り 出オフセット (mm)
    float velocity_turn135out;        ///< 135°出 速度 (mm/s)
    float alpha_turn135out;           ///< 135°出 ハンドリング
    float angle_turn135out;           ///< 135°出 角度 (度)
    float dist_turn135out_in;         ///< 135°出 入りオフセット (mm)
    float dist_turn135out_out;        ///< 135°出 出オフセット (mm)
    // ファン
    uint16_t fan_power;               ///< ファン出力 (0-1000)
    // 経路生成（makePath）引数設定
    int makepath_type_case3;          ///< case3 用 makePath 引数（将来拡張に備えて int）
    int makepath_type_case47;         ///< case4〜7 用 makePath 引数（将来拡張に備えて int）
} ShortestRunModeParams_t;

/**
 * @brief ケース個別（直線＋壁制御＋経路重み）パラメータ
 */
typedef struct {
    // 直線
    float acceleration_straight;      ///< 直線加速度 (mm/s^2)
    float acceleration_straight_dash; ///< 二段階直線加速度 (mm/s^2)
    float velocity_straight;          ///< 直線速度 (mm/s)
    // 斜め直線
    float acceleration_d_straight;      ///< 斜め直線加速度 (mm/s^2)
    float acceleration_d_straight_dash; ///< 斜め直線 二段階加速度 (mm/s^2)
    float velocity_d_straight;          ///< 斜め直線速度 (mm/s)
    // 壁制御
    float kp_wall;                    ///< 壁制御比例ゲイン
    float kp_diagonal;                ///< 斜め直進用 壁制御比例ゲイン
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

// ケース個別パラメータ（case3..N の順、要素数は実装依存）
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode2[];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode3[];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode4[];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode5[];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode6[];
extern const ShortestRunCaseParams_t shortestRunCaseParamsMode7[];

#endif // SHORTEST_RUN_PARAMS_H
