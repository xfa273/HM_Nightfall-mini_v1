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

// Helper loaders: apply case/mode parameters to runtime globals
static void apply_case_params_mode2_idx(int idx) {
    const ShortestRunCaseParams_t *c = &shortestRunCaseParamsMode2[idx];
    acceleration_straight = c->acceleration_straight;
    acceleration_straight_dash = c->acceleration_straight_dash;
    velocity_straight = c->velocity_straight;
    acceleration_d_straight = c->acceleration_d_straight;
    acceleration_d_straight_dash = c->acceleration_d_straight_dash;
    velocity_d_straight = c->velocity_d_straight;
    kp_wall = c->kp_wall;
    kp_diagonal = c->kp_diagonal;
}

static void apply_turn_normal_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turn90 = m->velocity_turn90;
    alpha_turn90 = m->alpha_turn90;
    acceleration_turn = m->acceleration_turn;
    dist_offset_in = m->dist_offset_in;
    dist_offset_out = m->dist_offset_out;
    val_offset_in = m->val_offset_in;
    angle_turn_90 = m->angle_turn_90;
    dist_wall_end = m->dist_wall_end;
}

static void apply_turn_large90_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_l_turn_90 = m->velocity_l_turn_90;
    alpha_l_turn_90 = m->alpha_l_turn_90;
    angle_l_turn_90 = m->angle_l_turn_90;
    dist_l_turn_in_90 = m->dist_l_turn_in_90;
    dist_l_turn_out_90 = m->dist_l_turn_out_90;
}

static void apply_turn_large180_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_l_turn_180 = m->velocity_l_turn_180;
    alpha_l_turn_180 = m->alpha_l_turn_180;
    angle_l_turn_180 = m->angle_l_turn_180;
    dist_l_turn_in_180 = m->dist_l_turn_in_180;
    dist_l_turn_out_180 = m->dist_l_turn_out_180;
}

static void apply_turn_d45in_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turn45in = m->velocity_turn45in;
    alpha_turn45in = m->alpha_turn45in;
    angle_turn45in = m->angle_turn45in;
    dist_turn45in_in = m->dist_turn45in_in;
    dist_turn45in_out = m->dist_turn45in_out;
}

static void apply_turn_d45out_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turn45out = m->velocity_turn45out;
    alpha_turn45out = m->alpha_turn45out;
    angle_turn45out = m->angle_turn45out;
    dist_turn45out_in = m->dist_turn45out_in;
    dist_turn45out_out = m->dist_turn45out_out;
}

static void apply_turn_v90_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turnV90 = m->velocity_turnV90;
    alpha_turnV90 = m->alpha_turnV90;
    angle_turnV90 = m->angle_turnV90;
    dist_turnV90_in = m->dist_turnV90_in;
    dist_turnV90_out = m->dist_turnV90_out;
}

static void apply_turn_d135in_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turn135in = m->velocity_turn135in;
    alpha_turn135in = m->alpha_turn135in;
    angle_turn135in = m->angle_turn135in;
    dist_turn135in_in = m->dist_turn135in_in;
    dist_turn135in_out = m->dist_turn135in_out;
}

static void apply_turn_d135out_mode2(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams2;
    velocity_turn135out = m->velocity_turn135out;
    alpha_turn135out = m->alpha_turn135out;
    angle_turn135out = m->angle_turn135out;
    dist_turn135out_in = m->dist_turn135out_in;
    dist_turn135out_out = m->dist_turn135out_out;
}

void mode2() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 調整モード選択（0..7）

            printf("Mode 2-0 Turn Tuning (sub 0..7).\n");

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            // 直線パラメータは、通常=case3(index 2)、斜め系=case8(index 7) を使用（case1/2 追加に伴う再マップ）
            const int idx_normal = 2; // case3
            const int idx_diag   = 7; // case8

            switch (sub) {
            case 0: // 通常ターン
                apply_case_params_mode2_idx(idx_normal);
                apply_turn_normal_mode2();
                printf("Loaded params: normal turn (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                half_sectionA(velocity_turn90);
                turn_R90(0);
                half_sectionD(0);

                led_flash(5);
                drive_stop();
                break;
            case 1: // 90deg大回り
                apply_case_params_mode2_idx(idx_normal);
                apply_turn_large90_mode2();
                printf("Loaded params: large 90deg (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                half_sectionA(velocity_l_turn_90);
                l_turn_R90();
                half_sectionD(0);

                led_flash(5);
                drive_stop();
                break;
            case 2: // 180deg大回り
                apply_case_params_mode2_idx(idx_normal);
                apply_turn_large180_mode2();
                printf("Loaded params: large 180deg (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                half_sectionA(velocity_l_turn_180);
                l_turn_R180(0);
                half_sectionD(0);

                led_flash(5);
                drive_stop();
                break;
            case 3: // 45deg 入り
                apply_case_params_mode2_idx(idx_diag);
                apply_turn_d45in_mode2();
                printf("Loaded params: diag 45-in (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                half_sectionA(velocity_turn45in);
                turn_R45_In();
                run_diagonal(1,0);

                led_flash(5);
                drive_stop();
                break;
            case 4: // 45deg 出
                apply_case_params_mode2_idx(idx_diag);
                apply_turn_d45out_mode2();
                printf("Loaded params: diag 45-out (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                run_diagonal(1,velocity_turn45out);
                turn_L45_Out();
                run_diagonal(1,0);

                led_flash(5);
                drive_stop();
                break;
            case 5: // V90
                apply_case_params_mode2_idx(idx_diag);
                apply_turn_v90_mode2();
                printf("Loaded params: diag V90 (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                run_diagonal(1,velocity_turnV90);
                turn_LV90();
                run_diagonal(1,0);

                led_flash(5);
                drive_stop();
                break;
            case 6: // 135deg 入り
                apply_case_params_mode2_idx(idx_diag);
                apply_turn_d135in_mode2();
                printf("Loaded params: diag 135-in (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                half_sectionA(velocity_turn135in);
                turn_R135_In();
                run_diagonal(1,0);

                led_flash(5);
                drive_stop();
                break;
            case 7: // 135deg 出
                apply_case_params_mode2_idx(idx_diag);
                apply_turn_d135out_mode2();
                printf("Loaded params: diag 135-out (mode2).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                run_diagonal(1,velocity_turn135out);
                turn_L135_Out();
                run_diagonal(1,0);

                led_flash(5);
                drive_stop();
                break;
            case 8: // Straight test using case1 params (index0)
                // 直進テスト: mode2 の case1 で使用されるパラメータを参照
                apply_case_params_mode2_idx(0); // case1 -> index 0
                printf("Loaded params: straight test (mode2, case8 -> case1 params).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                // ログ開始（速度プロファイル）
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());

                // 台形加減速: 3区画=270mm を最短走行同様に実行
                run_trapezoid_distance_mm(6.0f * DIST_HALF_SEC, velocity_straight);

                // ログ停止
                log_stop();

                drive_stop();

                // センサEnter待ちでログ出力
                printf("[mode2-case8] Press sensor ENTER (FR>1500 & FL<600) to print logs...\n");
                while (1) {
                    if (ad_fr > 1500 && ad_fl < 600) {
                        break;
                    }
                    HAL_Delay(50);
                }
                log_print_all();

                led_flash(5);

                break;
            case 9: // Straight test using case7 params (index6)
                // 直進テスト: mode2 の case7 で使用されるパラメータを参照
                apply_case_params_mode2_idx(6); // case7 -> index 6
                printf("Loaded params: straight test (mode2, case9 -> case7 params).\n");

                velocity_interrupt = 0;

                led_flash(10);

                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                // ログ開始（速度プロファイル）
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());

                // 台形加減速: 3区画=270mm を最短走行同様に実行
                run_trapezoid_distance_mm(6.0f * DIST_HALF_SEC, velocity_straight);

                // ログ停止
                log_stop();

                drive_stop();

                // センサEnter待ちでログ出力
                printf("[mode2-case9] Press sensor ENTER (FR>1500 & FL<600) to print logs...\n");
                while (1) {
                    if (ad_fr > 1500 && ad_fl < 600) {
                        break;
                    }
                    HAL_Delay(50);
                }
                log_print_all();

                led_flash(5);

                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            // 動作内容はユーザー側で実装予定のため、ここでは読み込みのみ
            break;
        }

        case 1:
            run_shortest(2, 1);
            break;

        case 2:
            run_shortest(2, 2);
            break;

        case 3:
            run_shortest(2, 4);
            break;

        case 4:
            run_shortest(2, 4);
            break;

        case 5:
            run_shortest(2, 5);
            break;

        case 6:
            run_shortest(2, 6);
            break;

        case 7:
            run_shortest(2, 7);
            break;
        case 8:
            run_shortest(2, 8);
            break;
        case 9:
            run_shortest(2, 9);
            break;
        }
    }
}
