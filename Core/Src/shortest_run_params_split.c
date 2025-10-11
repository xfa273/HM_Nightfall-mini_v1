#include "../Inc/shortest_run_params.h"

// ========================= Mode 2 =========================
const ShortestRunModeParams_t shortestRunModeParams2 = {
    .velocity_turn90    = 300.0f,
    .alpha_turn90       = 9000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 8.0f,
    .dist_offset_out    = 15.5f,
    .val_offset_in      = 1750.0f,
    .angle_turn_90      = 89.5f,
    .velocity_l_turn_90 = 500.0f,
    .alpha_l_turn_90    = 4250.0f,
    .angle_l_turn_90    = 89.0f,
    .dist_l_turn_in_90  = 5.0f,
    .dist_l_turn_out_90 = 12.7f,
    .velocity_l_turn_180= 500.0f,
    .alpha_l_turn_180   = 4640.0f,
    .angle_l_turn_180   = 179.0f,
    .dist_l_turn_in_180 = 12.0f,
    .dist_l_turn_out_180= 19.0f,
    .fan_power          = 0,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode2[5] = {
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 1500.0f, .kp_wall = 0.05f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};

// ========================= Mode 3 =========================
const ShortestRunModeParams_t shortestRunModeParams3 = {
    .velocity_turn90    = 600.0f,
    .alpha_turn90       = 33900.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 5.0f,
    .dist_offset_out    = 24.0f,
    .val_offset_in      = 1100.0f,
    .angle_turn_90      = 85.0f,
    .velocity_l_turn_90 = 1000.0f,
    .alpha_l_turn_90    = 22000.0f,
    .angle_l_turn_90    = 88.0f,
    .dist_l_turn_in_90  = 2.0f,
    .dist_l_turn_out_90 = 42.0f,
    .velocity_l_turn_180= 1000.0f,
    .alpha_l_turn_180   = 16850.0f,
    .angle_l_turn_180   = 176.0f,
    .dist_l_turn_in_180 = 3.0f,
    .dist_l_turn_out_180= 45.0f,
    .fan_power          = 600,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode3[5] = {
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.05f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 10000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};

// ========================= Mode 4 =========================
const ShortestRunModeParams_t shortestRunModeParams4 = {
    .velocity_turn90    = 1000.0f,
    .alpha_turn90       = 22400.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 10.0f,
    .dist_offset_out    = 52.0f,
    .val_offset_in      = 1100.0f,
    .angle_turn_90      = 88.0f,
    .velocity_l_turn_90 = 1300.0f,
    .alpha_l_turn_90    = 7800.0f,
    .angle_l_turn_90    = 88.0f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 48.0f,
    .velocity_l_turn_180= 1300.0f,
    .alpha_l_turn_180   = 7500.0f,
    .angle_l_turn_180   = 178.0f,
    .dist_l_turn_in_180 = 0.0f,
    .dist_l_turn_out_180= 85.0f,
    .fan_power          = 200,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode4[5] = {
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.10f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 1300.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 15000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 15000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 20000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};

// ========================= Mode 5 =========================
const ShortestRunModeParams_t shortestRunModeParams5 = {
    .velocity_turn90    = 1200.0f,
    .alpha_turn90       = 31150.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 10.0f,
    .dist_offset_out    = 35.0f,
    .val_offset_in      = 1160.0f,
    .angle_turn_90      = 84.0f,
    .velocity_l_turn_90 = 1700.0f,
    .alpha_l_turn_90    = 17000.0f,
    .angle_l_turn_90    = 79.5f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 89.0f,
    .velocity_l_turn_180= 1500.0f,
    .alpha_l_turn_180   = 11000.0f,
    .angle_l_turn_180   = 175.0f,
    .dist_l_turn_in_180 = 0.0f,
    .dist_l_turn_out_180= 105.0f,
    .fan_power          = 200,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode5[5] = {
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 13000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.13f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 5555.6f, .acceleration_straight_dash = 13000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 8000.0f,  .acceleration_straight_dash = 20000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 8000.0f,  .acceleration_straight_dash = 23000.0f,
        .velocity_straight = 4500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 8000.0f,  .acceleration_straight_dash = 26000.0f,
        .velocity_straight = 4800.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};

// ========================= Mode 6 =========================
const ShortestRunModeParams_t shortestRunModeParams6 = {
    .velocity_turn90    = 1400.0f,
    .alpha_turn90       = 50000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 10.0f,
    .dist_offset_out    = 58.0f,
    .val_offset_in      = 1050.0f,
    .angle_turn_90      = 80.5f,
    .velocity_l_turn_90 = 2000.0f,
    .alpha_l_turn_90    = 19200.0f,
    .angle_l_turn_90    = 89.5f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 67.0f,
    .velocity_l_turn_180= 2000.0f,
    .alpha_l_turn_180   = 20500.0f,
    .angle_l_turn_180   = 179.0f,
    .dist_l_turn_in_180 = 0.0f,
    .dist_l_turn_out_180= 138.0f,
    .fan_power          = 200,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode6[5] = {
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.13f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4500.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4800.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 5000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};

// ========================= Mode 7 =========================
const ShortestRunModeParams_t shortestRunModeParams7 = {
    .velocity_turn90    = 1400.0f,
    .alpha_turn90       = 50000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 10.0f,
    .dist_offset_out    = 58.0f,
    .val_offset_in      = 1050.0f,
    .angle_turn_90      = 80.5f,
    .velocity_l_turn_90 = 2200.0f,
    .alpha_l_turn_90    = 28500.0f,
    .angle_l_turn_90    = 85.0f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 101.0f,
    .velocity_l_turn_180= 2200.0f,
    .alpha_l_turn_180   = 24000.0f,
    .angle_l_turn_180   = 176.8f,
    .dist_l_turn_in_180 = 0.0f,
    .dist_l_turn_out_180= 169.0f,
    .fan_power          = 200,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode7[5] = {
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 2300.0f, .kp_wall = 0.13f,
        .straight_weight = 0, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 2300.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 26000.0f,
        .velocity_straight = 4800.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 5000.0f, .kp_wall = 0.05f,
        .straight_weight = 2, .diagonal_weight = 0
    },
};
