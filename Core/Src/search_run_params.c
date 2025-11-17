#include "../Inc/search_run_params.h"

// 探索用共通パラメータ（従来の mode1 基準値を採用）
const SearchRunParams_t searchRunParams = {
    // 直線系
    .acceleration_straight      = 1000.0f,
    .acceleration_straight_dash = 0.0f,
    .velocity_straight          = 1000.0f,

    // 小回り90°
    .velocity_turn90   = 300.0f,
    .alpha_turn90      = 8080.0f,
    .acceleration_turn = 0.0f,

    // オフセット・角度
    .dist_offset_in  = 15.0f,
    .dist_offset_out = 26.0f,
    .angle_turn_90   = 89.7f,

    // 走行中の推定前壁距離（mm）。正の値で有効化、0以下なら従来式にフォールバック
    // 例: dist_offset_in=10mm だが走行中は 15mm で辻褄が合うなら 15.0f を設定
    .front_dist_turn_start_moving = 21.0f,

    // 壁切れ後追従距離
    .dist_wall_end   = 46.0f,

    // 壁制御
    .kp_wall = 0.65f
};
