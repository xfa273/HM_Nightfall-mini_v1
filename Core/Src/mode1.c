/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: // LED全部点灯

            printf("Mode 1-0.\n");

            while (1) {
                HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
            }

            break;

        case 8: // 足立法 ゴール到達で終了 300mm/s

            printf("Mode 1-8 (Goal Stop).\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

        case 1: // 足立法全面探索 300mm/s

            printf("Mode 1-1.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 9;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

        case 2: // 足立法全面探索 300mm/s

            printf("Mode 1-2.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

        case 3: // 足立法全面探索 300mm/s しきい値高め

            printf("Mode 1-3.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 8;   // 8
            dist_offset_out = 16.0; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.1;

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

        case 4: // // 足立法全面探索 300mm/s  しきい値低め
            printf("Mode 1-4.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 8;   // 8
            dist_offset_out = 16.0; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

        case 5: // 吸引探索 600mm/s

            printf("Mode 1-5.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 4000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 600;
            alpha_turn90 = 33900;
            acceleration_turn = 0;
            dist_offset_in = 8;   // 8
            dist_offset_out = 19.5; // 15.5
            val_offset_in = 1100;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.05;
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

        case 6: // まずゴール探索→保存→全面探索（300mm/s）

            printf("Mode 1-6 (Goal->Save->Full Explore).\n");

            // ===== 走行パラメータ（case 2 と同一） =====
            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 17.0; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

        case 7: // ターン調整 900mm/s

            printf("Mode 1-7.\n");

            break;
        }
    }
}
