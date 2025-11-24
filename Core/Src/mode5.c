/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"

// Helper loaders: apply case/mode parameters to runtime globals (mode5)
static void apply_case_params_mode5_idx(int idx) {
    const ShortestRunCaseParams_t *c = &shortestRunCaseParamsMode5[idx];
    acceleration_straight = c->acceleration_straight;
    acceleration_straight_dash = c->acceleration_straight_dash;
    velocity_straight = c->velocity_straight;
    // mode5 のケースでは対角直線パラメータは未使用
    kp_wall = c->kp_wall;
}

static void apply_turn_normal_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turn90 = m->velocity_turn90;
    alpha_turn90 = m->alpha_turn90;
    acceleration_turn = m->acceleration_turn;
    dist_offset_in = m->dist_offset_in;
    dist_offset_out = m->dist_offset_out;
    val_offset_in = m->val_offset_in;
    angle_turn_90 = m->angle_turn_90;
    dist_wall_end = m->dist_wall_end;
}

static void apply_turn_large90_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_l_turn_90 = m->velocity_l_turn_90;
    alpha_l_turn_90 = m->alpha_l_turn_90;
    angle_l_turn_90 = m->angle_l_turn_90;
    dist_l_turn_in_90 = m->dist_l_turn_in_90;
    dist_l_turn_out_90 = m->dist_l_turn_out_90;
}

static void apply_turn_large180_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_l_turn_180 = m->velocity_l_turn_180;
    alpha_l_turn_180 = m->alpha_l_turn_180;
    angle_l_turn_180 = m->angle_l_turn_180;
    dist_l_turn_in_180 = m->dist_l_turn_in_180;
    dist_l_turn_out_180 = m->dist_l_turn_out_180;
}

static void apply_turn_d45in_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turn45in = m->velocity_turn45in;
    alpha_turn45in = m->alpha_turn45in;
    angle_turn45in = m->angle_turn45in;
    dist_turn45in_in = m->dist_turn45in_in;
    dist_turn45in_out = m->dist_turn45in_out;
}

static void apply_turn_d45out_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turn45out = m->velocity_turn45out;
    alpha_turn45out = m->alpha_turn45out;
    angle_turn45out = m->angle_turn45out;
    dist_turn45out_in = m->dist_turn45out_in;
    dist_turn45out_out = m->dist_turn45out_out;
}

static void apply_turn_v90_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turnV90 = m->velocity_turnV90;
    alpha_turnV90 = m->alpha_turnV90;
    angle_turnV90 = m->angle_turnV90;
    dist_turnV90_in = m->dist_turnV90_in;
    dist_turnV90_out = m->dist_turnV90_out;
}

static void apply_turn_d135in_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turn135in = m->velocity_turn135in;
    alpha_turn135in = m->alpha_turn135in;
    angle_turn135in = m->angle_turn135in;
    dist_turn135in_in = m->dist_turn135in_in;
    dist_turn135in_out = m->dist_turn135in_out;
}

static void apply_turn_d135out_mode5(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams5;
    velocity_turn135out = m->velocity_turn135out;
    alpha_turn135out = m->alpha_turn135out;
    angle_turn135out = m->angle_turn135out;
    dist_turn135out_in = m->dist_turn135out_in;
    dist_turn135out_out = m->dist_turn135out_out;
}

void mode5() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 調整モード選択（0..9）

            printf("Mode 5-0 Turn/Diagonal/Straight Test (sub 0..9).\n");

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            // 直線パラメータのデフォルト（mode5 は5ケース）
            const int idx_normal = 2; // case3 相当
            const int idx_diag   = 2; // 斜めも同一ベースを使用

            switch (sub) {
            case 0: // 通常ターン（小回りR90）
                apply_case_params_mode5_idx(idx_normal);
                apply_turn_normal_mode5();
                printf("Loaded params: normal turn (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 300; // 右小回り
                run();
                drive_fan(0);
                break;
            case 1: // 90deg大回り
                apply_case_params_mode5_idx(idx_normal);
                apply_turn_large90_mode5();
                printf("Loaded params: large 90deg (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 501; // L-R90
                run();
                drive_fan(0);
                break;
            case 2: // 180deg大回り
                apply_case_params_mode5_idx(idx_normal);
                apply_turn_large180_mode5();
                printf("Loaded params: large 180deg (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 502; // L-R180
                run();
                drive_fan(0);
                break;
            case 3: // 45deg 入り
                apply_case_params_mode5_idx(idx_diag);
                apply_turn_d45in_mode5();
                printf("Loaded params: diag 45-in (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 701;   // 右45°入り
                path[1] = 1000+1; // 斜めS1
                run();
                drive_fan(0);
                break;
            case 4: // 45deg 出
                apply_case_params_mode5_idx(idx_diag);
                apply_turn_d45out_mode5();
                printf("Loaded params: diag 45-out (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 1000+1; // 斜めS1
                path[1] = 704;    // 左45°出
                path[2] = 1000+1; // 斜めS1
                run();
                drive_fan(0);
                break;
            case 5: // V90
                apply_case_params_mode5_idx(idx_diag);
                apply_turn_v90_mode5();
                printf("Loaded params: diag V90 (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 1000+1; // 斜めS1
                path[1] = 802;    // 左V90
                path[2] = 1000+1; // 斜めS1
                run();
                drive_fan(0);
                break;
            case 6: // 135deg 入り
                apply_case_params_mode5_idx(idx_diag);
                apply_turn_d135in_mode5();
                printf("Loaded params: diag 135-in (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 901;    // 右135°入り
                path[1] = 1000+1; // 斜めS1
                run();
                drive_fan(0);
                break;
            case 7: // 135deg 出
                apply_case_params_mode5_idx(idx_diag);
                apply_turn_d135out_mode5();
                printf("Loaded params: diag 135-out (mode5).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 1000+1; // 斜めS1
                path[1] = 904;    // 左135°出
                path[2] = 1000+1; // 斜めS1
                run();
                drive_fan(0);
                break;
            case 8: { // Straight test (slow)
                apply_case_params_mode5_idx(0);
                kp_wall = 0.0f;
                printf("Loaded params: straight test (mode5, case8 -> case1 params).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_DISTANCE);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3; // S3
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode5-case8] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break; }
            case 9: { // Straight test (fast)
                apply_case_params_mode5_idx(4);
                kp_wall = 0.0f;
                printf("Loaded params: straight test (mode5, case9 -> case5 params).\n");
                drive_fan(shortestRunModeParams5.fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3; // S3
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode5-case9] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break; }
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            // 動作内容はユーザー側で実装予定のため、ここでは読み込みのみ
            break;
        }

        case 1: //

            printf("Mode 5-1 Large Turn 90deg.\n");

            // 直線
            acceleration_straight = 8000;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 90°大回りターン
            velocity_l_turn_90 = 1800;
            alpha_l_turn_90 = 12000;
            angle_l_turn_90 = 88;
            dist_l_turn_out_90 = 38;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(300);

            led_flash(20);

            half_sectionA(1800);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 2:

            printf("Mode 4-2 Large Turn 180deg.\n");

            // 直線
            acceleration_straight = 8000;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 180°大回りターン
            velocity_l_turn_180 = 1600;
            alpha_l_turn_180 = 10700;
            angle_l_turn_180 = 177;
            dist_l_turn_out_180 = 65;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(300);

            led_flash(20);

            half_sectionA(1600);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 3:
            printf("Mode 5-3.\n");
            run_shortest(5, 3);
            break;

        case 4:
            run_shortest(5, 4);
            break;

        case 5:
            run_shortest(5, 5);
            break;

        case 6:
            run_shortest(5, 6);
            break;

        case 7:
            run_shortest(5, 7);
            break;

        case 8:

            printf("Mode 5-8.\n");

            // 経路の重み
            straight_weight = 3; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunCaseParams_t *pc = &shortestRunCaseParamsMode5[4];
                const ShortestRunModeParams_t *pm = &shortestRunModeParams5;
                // 直線（caseごと）
                acceleration_straight      = pc->acceleration_straight;
                acceleration_straight_dash = pc->acceleration_straight_dash;
                velocity_straight          = pc->velocity_straight;
                // ターン（mode共通）
                velocity_turn90            = pm->velocity_turn90;
                alpha_turn90               = pm->alpha_turn90;
                acceleration_turn          = pm->acceleration_turn;
                dist_offset_in             = pm->dist_offset_in;
                dist_offset_out            = pm->dist_offset_out;
                val_offset_in              = pm->val_offset_in;
                angle_turn_90              = pm->angle_turn_90;
                velocity_l_turn_90         = pm->velocity_l_turn_90;
                alpha_l_turn_90            = pm->alpha_l_turn_90;
                angle_l_turn_90            = pm->angle_l_turn_90;
                dist_l_turn_out_90         = pm->dist_l_turn_out_90;
                velocity_l_turn_180        = pm->velocity_l_turn_180;
                alpha_l_turn_180           = pm->alpha_l_turn_180;
                angle_l_turn_180           = pm->angle_l_turn_180;
                dist_l_turn_out_180        = pm->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.3;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(300);

            led_flash(3);

            run_shortest(5, 5);

            drive_fan(0);

            led_wait();

            break;

        case 9:

            printf("Mode 5-9.\n");

            // 経路の重み
            straight_weight = 1; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            // 直線
            acceleration_straight = 8000;
            acceleration_straight_dash = 26000;
            velocity_straight = 4800;
            // ターン
            velocity_turn90 = 1200;
            alpha_turn90 = 31150;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
            val_offset_in = 1160;
            angle_turn_90 = 84;
            // 90°大回りターン
            velocity_l_turn_90 = 1700;
            alpha_l_turn_90 = 17000;
            angle_l_turn_90 = 79.5;
            dist_l_turn_in_90 = 0;
            dist_l_turn_out_90 = 89;
            // 180°大回りターン
            velocity_l_turn_180 = 1500;
            alpha_l_turn_180 = 11000;
            angle_l_turn_180 = 175;
            dist_l_turn_in_180 = 0;
            dist_l_turn_out_180 = 105;
            // 壁制御とケツ当て
            kp_wall = 0.3;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(300);

            led_flash(3);

            run_shortest(5, 5);

            drive_fan(0);

            led_wait();

            break;
        }
    }
}
