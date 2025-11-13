/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/search_run_params.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"
#include <math.h>

// 探索パラメータの簡易調整用オーバーライド（未設定は NaN）
static float s_override_velocity_straight = NAN;
static float s_override_accel_straight = NAN;
static float s_override_kp_wall = NAN;
static float s_override_dist_wall_end = NAN;

static void print_search_params_current(void) {
    printf("[Search Params] vel=%.1f, acc=%.1f/%.1f, kp_wall=%.3f, off_in=%.1f, off_out=%.1f, val_in=%.0f, ang=%.1f, wall_end=%.1f\n",
           velocity_straight, acceleration_straight, acceleration_straight_dash, kp_wall,
           dist_offset_in, dist_offset_out, val_offset_in, angle_turn_90, dist_wall_end);
}

// --- 以下、最短走行のターン調整と同様のヘルパ（mode4相当の読み替え） ---
static void apply_case_params_mode4_like(int idx) {
    const ShortestRunCaseParams_t *c = &shortestRunCaseParamsMode4[idx];
    acceleration_straight = c->acceleration_straight;
    acceleration_straight_dash = c->acceleration_straight_dash;
    velocity_straight = c->velocity_straight;
    acceleration_d_straight = c->acceleration_d_straight;
    acceleration_d_straight_dash = c->acceleration_d_straight_dash;
    velocity_d_straight = c->velocity_d_straight;
    kp_wall = c->kp_wall;
    kp_diagonal = c->kp_diagonal;
}

static void apply_turn_normal_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn90 = m->velocity_turn90;
    alpha_turn90 = m->alpha_turn90;
    acceleration_turn = m->acceleration_turn;
    dist_offset_in = m->dist_offset_in;
    dist_offset_out = m->dist_offset_out;
    val_offset_in = m->val_offset_in;
    angle_turn_90 = m->angle_turn_90;
    dist_wall_end = m->dist_wall_end;
}

static void apply_turn_large90_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_l_turn_90 = m->velocity_l_turn_90;
    alpha_l_turn_90 = m->alpha_l_turn_90;
    angle_l_turn_90 = m->angle_l_turn_90;
    dist_l_turn_in_90 = m->dist_l_turn_in_90;
    dist_l_turn_out_90 = m->dist_l_turn_out_90;
}

static void apply_turn_large180_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_l_turn_180 = m->velocity_l_turn_180;
    alpha_l_turn_180 = m->alpha_l_turn_180;
    angle_l_turn_180 = m->angle_l_turn_180;
    dist_l_turn_in_180 = m->dist_l_turn_in_180;
    dist_l_turn_out_180 = m->dist_l_turn_out_180;
}

static void apply_turn_d45in_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn45in = m->velocity_turn45in;
    alpha_turn45in = m->alpha_turn45in;
    angle_turn45in = m->angle_turn45in;
    dist_turn45in_in = m->dist_turn45in_in;
    dist_turn45in_out = m->dist_turn45in_out;
}

static void apply_turn_d45out_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn45out = m->velocity_turn45out;
    alpha_turn45out = m->alpha_turn45out;
    angle_turn45out = m->angle_turn45out;
    dist_turn45out_in = m->dist_turn45out_in;
    dist_turn45out_out = m->dist_turn45out_out;
}

static void apply_turn_v90_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turnV90 = m->velocity_turnV90;
    alpha_turnV90 = m->alpha_turnV90;
    angle_turnV90 = m->angle_turnV90;
    dist_turnV90_in = m->dist_turnV90_in;
    dist_turnV90_out = m->dist_turnV90_out;
}

static void apply_turn_d135in_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn135in = m->velocity_turn135in;
    alpha_turn135in = m->alpha_turn135in;
    angle_turn135in = m->angle_turn135in;
    dist_turn135in_in = m->dist_turn135in_in;
    dist_turn135in_out = m->dist_turn135in_out;
}

static void apply_turn_d135out_mode4_like(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn135out = m->velocity_turn135out;
    alpha_turn135out = m->alpha_turn135out;
    angle_turn135out = m->angle_turn135out;
    dist_turn135out_in = m->dist_turn135out_in;
    dist_turn135out_out = m->dist_turn135out_out;
}

// Helper loader: apply exploration common params
static void apply_search_run_params(void) {
    const SearchRunParams_t *p = &searchRunParams;
    // 直線
    acceleration_straight = p->acceleration_straight;
    acceleration_straight_dash = p->acceleration_straight_dash;
    velocity_straight = p->velocity_straight;
    // 壁制御
    kp_wall = p->kp_wall;
    // ターン・オフセット・角度
    velocity_turn90 = p->velocity_turn90;
    alpha_turn90 = p->alpha_turn90;
    acceleration_turn = p->acceleration_turn;
    dist_offset_in = p->dist_offset_in;
    dist_offset_out = p->dist_offset_out;
    val_offset_in = p->val_offset_in;
    angle_turn_90 = p->angle_turn_90;
    // 壁切れ後追従
    dist_wall_end = p->dist_wall_end;

    // オーバーライドを適用
    if (!isnan(s_override_velocity_straight)) {
        if (s_override_velocity_straight < 100.0f) s_override_velocity_straight = 100.0f;
        if (s_override_velocity_straight > 3000.0f) s_override_velocity_straight = 3000.0f;
        velocity_straight = s_override_velocity_straight;
    }
    if (!isnan(s_override_accel_straight)) {
        if (s_override_accel_straight < 200.0f) s_override_accel_straight = 200.0f;
        if (s_override_accel_straight > 20000.0f) s_override_accel_straight = 20000.0f;
        acceleration_straight = s_override_accel_straight;
    }
    if (!isnan(s_override_kp_wall)) {
        if (s_override_kp_wall < 0.0f) s_override_kp_wall = 0.0f;
        if (s_override_kp_wall > 0.5f) s_override_kp_wall = 0.5f;
        kp_wall = s_override_kp_wall;
    }
    if (!isnan(s_override_dist_wall_end)) {
        if (s_override_dist_wall_end < 0.0f) s_override_dist_wall_end = 0.0f;
        if (s_override_dist_wall_end > 60.0f) s_override_dist_wall_end = 60.0f;
        dist_wall_end = s_override_dist_wall_end;
    }
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 調整サブモード: 最短走行のターン調整と同様

            printf("Mode 1-0 Turn Tuning (same as shortest).\n");
            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            // 通常=case3(index2)、斜め=case8(index7) を参照
            const int idx_normal = 2;
            const int idx_diag   = 7;

            switch (sub) {
            case 0: // 通常ターン
                apply_case_params_mode4_like(idx_normal);
                apply_turn_normal_mode4_like();
                printf("Loaded params: normal turn.\n");

                velocity_interrupt = 0;

                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());

                half_sectionA(velocity_turn90);
                turn_R90(0);
                half_sectionD(0);

                log_stop();
                led_flash(5);
                drive_fan(0);
                drive_stop();

                // センサEnter待ち（右前:角速度ログ, 左前:角度ログ）
                while (1) {
                    if (ad_fr > 1500) {
                        log_print_omega_all();
                        break;
                    } else if (ad_fl > 1500) {
                        log_print_angle_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            case 1: // 90deg大回り
                apply_case_params_mode4_like(idx_normal);
                apply_turn_large90_mode4_like();
                printf("Loaded params: large 90deg.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());

                run_straight(2, velocity_l_turn_90, 0);
                l_turn_R90();
                run_straight(1, 0, 0);

                log_stop();
                led_flash(5);
                drive_fan(0);
                drive_stop();

                while (1) {
                    if (ad_fr > 1500) {
                        log_print_omega_all();
                        break;
                    } else if (ad_fl > 1500) {
                        log_print_angle_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            case 2: // 180deg大回り
                apply_case_params_mode4_like(idx_normal);
                apply_turn_large180_mode4_like();
                printf("Loaded params: large 180deg.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());

                half_sectionA(velocity_l_turn_180);
                l_turn_R180(0);
                half_sectionD(0);

                log_stop();
                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            case 3: // 45deg 入り
                apply_case_params_mode4_like(idx_diag);
                apply_turn_d45in_mode4_like();
                printf("Loaded params: diag 45-in.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                half_sectionA(velocity_turn45in);
                turn_R45_In();
                run_diagonal(1,0);

                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            case 4: // 45deg 出
                apply_case_params_mode4_like(idx_diag);
                apply_turn_d45out_mode4_like();
                printf("Loaded params: diag 45-out.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                run_diagonal(1,velocity_turn45out);
                turn_L45_Out();
                run_diagonal(1,0);

                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            case 5: // V90
                apply_case_params_mode4_like(idx_diag);
                apply_turn_v90_mode4_like();
                printf("Loaded params: diag V90.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                run_diagonal(1,velocity_turnV90);
                turn_RV90();
                run_diagonal(1,0);

                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            case 6: // 135deg 入り
                apply_case_params_mode4_like(idx_diag);
                apply_turn_d135in_mode4_like();
                printf("Loaded params: diag 135-in.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                run_diagonal(1,velocity_turn135in);
                turn_R135_In();
                run_diagonal(1,0);

                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            case 7: // 135deg 出
                apply_case_params_mode4_like(idx_diag);
                apply_turn_d135out_mode4_like();
                printf("Loaded params: diag 135-out.\n");

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(5);
                drive_fan(shortestRunModeParams4.fan_power);
                led_flash(5);

                run_diagonal(1,velocity_turn135out);
                turn_L135_Out();
                run_diagonal(1,0);

                led_flash(5);
                drive_fan(0);
                drive_stop();
                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            break;
        }

        case 8:
            printf("Mode 1-8: (empty)\n");
            break;

        case 1:
            printf("Mode 1-1: (empty)\n");
            break;

        case 2:
            printf("Mode 1-2: (empty)\n");
            break;

        case 3:
            printf("Mode 1-3: (empty)\n");
            break;

        case 4:
            printf("Mode 1-4: (empty)\n");
            break;

        case 5:
            printf("Mode 1-5: (empty)\n");
            break;

        case 6:
            printf("Mode 1-6: (empty)\n");
            break;

        case 7:
            printf("Mode 1-7: (empty)\n");
            break;
        case 9:
            printf("Mode 1-9: (empty)\n");
            break;
        }
    }
}
