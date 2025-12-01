/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "sensor_distance.h"
#include "../Inc/logging.h"
#include "../Inc/search_run_params.h"



//============================================================
// 探索走行パラメータ適用ヘルパー関数
//============================================================
static void apply_search_params(int case_index)
{
    if (case_index < 0 || case_index >= 2) {
        printf("Error: Invalid case_index %d\n", case_index);
        return;
    }

    const SearchRunParams_t *params = &searchRunParams[case_index];

    // 直線パラメータ
    acceleration_straight = params->acceleration_straight;
    acceleration_straight_dash = params->acceleration_straight_dash;

    // ターンパラメータ
    velocity_turn90 = params->velocity_turn90;
    alpha_turn90 = params->alpha_turn90;
    acceleration_turn = params->acceleration_turn;
    dist_offset_in = params->dist_offset_in;
    dist_offset_out = params->dist_offset_out;
    val_offset_in = params->val_offset_in;
    angle_turn_90 = params->angle_turn_90;

    // 壁切れ後の追従距離
    dist_wall_end = params->dist_wall_end;

    // 壁制御パラメータ
    kp_wall = params->kp_wall;
    duty_setposition = params->duty_setposition;

    // センサパラメータ
    sensor_kx = params->sensor_kx;

    // フラグ
    MF.FLAG.WALL_ALIGN = params->wall_align_enable;

    printf("Applied search params case %d (velocity: %.0f mm/s)\n", 
           case_index + 1, params->velocity_turn90);
}

//============================================================
// 前壁追従（match_position連続実行）テスト
//  with_print!=0 でFR/FLのAD値と距離[mm]を定期表示
//  PUSHボタン押下で終了
//============================================================
static void front_follow_continuous(int with_print)
{
    printf("[FrontFollow] match_position continuous. Press PUSH to exit.\n");
    led_flash(3);

    // 側壁制御を無効化して前壁のみで合わせる
    MF.FLAG.CTRL = 0;
    kp_wall = 0.0f;

    // 走行開始準備
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();
    drive_start();

    uint32_t last_print = HAL_GetTick();

    while (1) {
        // 抜け条件：PUSHボタン
        if (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) {
            buzzer_enter(900);
            break;
        }

        // 前壁が見えているときに位置合わせを実行
        if (ad_fr > F_ALIGN_DETECT_THR && ad_fl > F_ALIGN_DETECT_THR) {
            match_position(0);
        } else {
            // 待機（見えていない間は停止）
            velocity_interrupt = 0;
            omega_interrupt = 0;
            HAL_Delay(50);
        }

        // 任意の表示
        if (with_print) {
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 200) {
                float d_fr = sensor_distance_from_fr(ad_fr);
                float d_fl = sensor_distance_from_fl(ad_fl);
                printf("FR=%u (%.1fmm), FL=%u (%.1fmm)\n",
                       (unsigned)ad_fr, d_fr, (unsigned)ad_fl, d_fl);
                last_print = now;
            }
        }

        HAL_Delay(10);
    }

    // 停止処理
    velocity_interrupt = 0;
    omega_interrupt = 0;
    drive_variable_reset();
    drive_stop();
    led_flash(2);
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // テストモード

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            switch (sub) {
            case 0:

                break;
            case 1:

                break;
            case 2:

                break;
            case 3:

                break;
            default:

                break;
            }

            break;
        }


        case 1: { // 標準速度で ゴール探索→全面探索

            printf("Mode 1-1: Standard speed (Goal->Full).\n");
            
            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

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

            // 一旦マップ保存
            store_map_in_eeprom();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true;
            search_end = false;
            adachi();

            led_wait();

            break;
        }

        case 2: // 標準速度で 最初から全面探索

            printf("Mode 1-2: Standard speed (Full from start).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // 最初から全面探索
            set_search_mode(SEARCH_MODE_FULL);
            search_end = false;
            adachi();

            led_wait();

            break;

        case 3: // 標準速度で ゴール探索→スタートへ帰り探索

            printf("Mode 1-3: Standard speed (Goal->Return to Start).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false;
            goal_x = GOAL_X; goal_y = GOAL_Y;
            search_end = false;
            adachi();

            // ゴール到達後に一度だけ安全に保存
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1;
                }
            }

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0;
            g_goal_is_start = true;
            goal_x = START_X; goal_y = START_Y;
            search_end = false;
            adachi();

            // 後処理
            g_goal_is_start = false;

            led_wait();

            break;

        case 4: // 標準速度で ゴール探索→ゴール到達で終了
            printf("Mode 1-4: Standard speed (Goal only).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi();

            led_wait();

            break;

        case 5: // 低速で ゴール探索→全面探索

            printf("Mode 1-5: Low speed (Goal->Full).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

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

            // 一旦マップ保存
            store_map_in_eeprom();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true;
            search_end = false;
            adachi();

            led_wait();

            break;

        case 6: // 低速で 最初から全面探索

            printf("Mode 1-6: Low speed (Full from start).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // 最初から全面探索
            set_search_mode(SEARCH_MODE_FULL);
            search_end = false;
            adachi();

            led_wait();

            break;

        case 7: // 低速で ゴール探索→スタートへ帰り探索

            printf("Mode 1-7: Low speed (Goal->Return to Start).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false;
            goal_x = GOAL_X; goal_y = GOAL_Y;
            search_end = false;
            adachi();

            // ゴール到達後に一度だけ安全に保存
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1;
                }
            }

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0;
            g_goal_is_start = true;
            goal_x = START_X; goal_y = START_Y;
            search_end = false;
            adachi();

            // 後処理
            g_goal_is_start = false;

            led_wait();

            break;

        case 8: // 低速で ゴール探索→ゴール到達で終了
            printf("Mode 1-8: Low speed (Goal only).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi();

            led_wait();

            break;
        }
    }
}
