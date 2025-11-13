/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/search_run_params.h"

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
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: // LED全部点灯

            printf("Mode 1-0.\n");

            break;

        case 8: // 足立法 ゴール到達で終了（探索共通パラメータ適用）

            printf("Mode 1-8 (Goal Stop).\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

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

            adachi();

            led_wait();

            break;

        case 1: // 足立法全面探索（探索共通パラメータ適用）

            printf("Mode 1-1.\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 1;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 2: // 足立法全面探索（探索共通パラメータ適用）

            printf("Mode 1-2.\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 3: // 足立法全面探索（しきい値高め、探索共通パラメータ適用）

            printf("Mode 1-3.\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.2;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 4: // 足立法全面探索（しきい値低め、探索共通パラメータ適用）
            printf("Mode 1-4.\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 0.9;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 5: // 吸引探索（探索共通パラメータ適用 + ファンON）

            printf("Mode 1-5.\n");

            MF.FLAG.RUNNING = 1;

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_fan(300);
            led_flash(3);

            drive_start();

            adachi();

            drive_fan(0);

            led_wait();

            break;

        case 6: // まずゴール探索→保存→全面探索（探索共通パラメータ適用）

            printf("Mode 1-6 (Goal->Save->Full Explore).\n");

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

            // 一旦マップ保存
            store_map_in_eeprom();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            // フル探索に切り替えた直後の「最初の停止での保存」を1回抑制
            g_suppress_first_stop_save = true;
            search_end = false;
            adachi();

            led_wait();

            break;

        case 7: // ゴール探索→保存→スタートへ復帰（探索共通パラメータ適用）

            printf("Mode 1-7 (Goal->Save->Return to Start).\n");

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

            // ゴール到達後に一度だけ安全に保存（Uターン時に保存済みなら二重保存を避ける）
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1; // 以後の自動保存を抑制
                }
            }

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
        }
    }
}
