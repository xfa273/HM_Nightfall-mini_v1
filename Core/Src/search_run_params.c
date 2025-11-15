#include "../Inc/search_run_params.h"

// 探索用共通パラメータ（従来の mode1 基準値を採用）
const SearchRunParams_t searchRunParams = {
    // 直線系
    .acceleration_straight      = 1000.0f,
    .acceleration_straight_dash = 0.0f,
    .velocity_straight          = 1000.0f,

    // 小回り90°
    .velocity_turn90   = 300.0f,
    .alpha_turn90      = 8300.0f,
    .acceleration_turn = 0.0f,

    // オフセット・角度
    .dist_offset_in  = 10.0f,
    .dist_offset_out = 20.0f,
    .val_offset_in   = 1750.0f,
    .angle_turn_90   = 89.4f,

    // 壁切れ後追従距離
    .dist_wall_end   = 40.0f,

    // 壁制御
    .kp_wall = 0.015f
};
