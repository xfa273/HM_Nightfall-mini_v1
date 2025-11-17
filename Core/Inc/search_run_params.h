#ifndef SEARCH_RUN_PARAMS_H
#define SEARCH_RUN_PARAMS_H

#include <stdint.h>

// 探索走行（足立法）用の共通パラメータ定義
// 最短走行とは独立に、探索に適した低速・安定重視の値をまとめる

typedef struct {
    // 直線系
    float acceleration_straight;      // [mm/s^2]
    float acceleration_straight_dash; // [mm/s^2]
    float velocity_straight;          // [mm/s]

    // 小回り90°（探索ターン）
    float velocity_turn90;            // [mm/s]
    float alpha_turn90;               // ハンドリング係数
    float acceleration_turn;          // [mm/s^2]

    // 進入/退出オフセット・角度
    float dist_offset_in;             // [mm]
    float dist_offset_out;            // [mm]
    float angle_turn_90;              // [deg]

    // 走行中の推定前壁距離（ターン開始時のしきい値に使用）。
    // 正の値を設定すると、その値[mm]を driveFWall の前壁距離しきい値に使用。
    // 0以下の場合は従来通り FRONT_DIST_AT_CELL_ENTRY_MM - dist_offset_in を用いる。
    float front_dist_turn_start_moving; // [mm]

    // 壁切れ後の直進距離（追従）
    float dist_wall_end;              // [mm]

    // 壁制御
    float kp_wall;                    // 探索向け壁制御比例
} SearchRunParams_t;

// 探索共通パラメータ（初期値は従来の mode1 基準値）
extern const SearchRunParams_t searchRunParams;

#endif // SEARCH_RUN_PARAMS_H
