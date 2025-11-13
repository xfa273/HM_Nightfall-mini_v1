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

// （mode4相当のヘルパは不要のため削除）

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
        case 0: { // 調整サブモード: サブcase1で通常ターンのみ（探索用共通パラメータ）

            printf("Mode 1-0 Turn Tuning (search params): sub=1 only.\n");
            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            switch (sub) {
            case 0:
                printf("(empty)\n");
                break;
            case 1: // 通常ターン 90deg（探索用共通パラメータ）
                // 探索用の共通パラメータを適用
                apply_search_run_params();
                // 調整モードでは壁制御を無効化
                kp_wall = 0.0f;

                velocity_interrupt = 0;

                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());

                half_sectionA(velocity_turn90);
                turn_R90(0);
                half_sectionD(0);

                log_stop();
                led_flash(5);
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
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                printf("(empty)\n");
                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            break;
        }

        case 1: // 探索: ゴールを目指し、到達で終了

            printf("Mode 1-1 (Explore: Goal Stop).\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // 準備
            led_flash(10);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);

            adachi();

            led_wait();

            break;

        case 2: // 探索: ゴールを目指し、到達後にスタートへ戻る

            printf("Mode 1-2 (Explore: Goal -> Return to Start).\n");

            // ===== 走行パラメータ（探索共通パラメータを適用） =====
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // ===== 事前準備 =====
            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false; // ゴールセルを到達判定
            goal_x = GOAL_X; goal_y = GOAL_Y; // 念のため明示
            search_end = false;
            adachi();

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0; // 復路ではゴール判定フラグに依存しない
            g_goal_is_start = true; // スタート座標を到達判定に使用
            goal_x = START_X; goal_y = START_Y; // 経路導出もスタートへ
            search_end = false;
            adachi();

            // 後処理
            g_goal_is_start = false; // 後続モードへの影響を避ける

            led_wait();

            break;

        case 3: // 探索: ゴール到達後に全面探索へ切り替え

            printf("Mode 1-3 (Explore: Goal -> Full Explore).\n");

            // ===== 走行パラメータ（探索共通パラメータを適用） =====
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // ===== 事前準備 =====
            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true; // 切替直後の最初の停止保存を1回スキップ
            search_end = false;
            adachi();

            led_wait();

            break;

        case 4: // 探索: 最初から全面探索

            printf("Mode 1-4 (Explore: Full).\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // 準備
            led_flash(10);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            // 全面探索モード
            set_search_mode(SEARCH_MODE_FULL);
            search_end = false;

            adachi();

            led_wait();

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

        case 8:
            printf("Mode 1-8: (empty)\n");
            break;

        case 9:
            printf("Mode 1-9: (empty)\n");
            break;
        }
    }
}
