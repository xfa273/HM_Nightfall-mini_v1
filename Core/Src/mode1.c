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

        case 1: // 足立法全面探索 600mm/s

            printf("Mode 1-1.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 1200; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 9000;
            acceleration_turn = 0;
            dist_offset_in = 9;
            dist_offset_out = 16.3; // 32
            val_offset_in = 1780;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 12;

            // 壁制御とケツ当て
            kp_wall = 0.02;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

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

        case 2: // 足立法全面探索 1000mm/s

            printf("Mode 1-2.\n");

            // 直線
            acceleration_straight = 4000;
            acceleration_straight_dash = 4000; // 5000
            // ターン
            velocity_turn90 = 600;
            alpha_turn90 = 33900;
            acceleration_turn = 0;
            dist_offset_in = 5;
            dist_offset_out = 24;
            val_offset_in = 1100;
            angle_turn_90 = 85;
            // 壁切れ後の距離
            dist_wall_end = 12;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            drive_fan(300);

            adachi();

            drive_fan(0);

            led_wait();

            break;

        case 3: // 足立法全面探索 1000mm/s しきい値高め

            printf("Mode 1-3.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 3000;
            velocity_straight = 4000;
            // ターン
            velocity_turn90 = 1000;
            alpha_turn90 = 22400;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 45;
            val_offset_in = 1100;
            angle_turn_90 = 88.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            thr_f_wall = 770;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.15;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            drive_fan(120);

            led_flash(10);

            adachi();

            led_wait();

            break;

        case 4: // // 足立法全面探索 1000mm/s  しきい値低め
            printf("Mode 1-4.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 3000;
            velocity_straight = 4000;
            // ターン
            velocity_turn90 = 1000;
            alpha_turn90 = 22400;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 45;
            val_offset_in = 1100;
            angle_turn_90 = 88.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            thr_f_wall = 770;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 0.85;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            drive_fan(120);

            led_flash(10);

            adachi();

            led_wait();

            break;

        case 5: // ターン調整 600mm/s

            printf("Mode 1-5.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2000;
            acceleration_straight_dash = 2000; // 5000
            // ターン
            velocity_turn90 = 600;
            alpha_turn90 = 10300;
            acceleration_turn = 0;
            dist_offset_in = 20;
            dist_offset_out = 28; // 32
            val_offset_in = 730;
            angle_turn_90 = 89;
            // 壁切れ後の距離
            dist_wall_end = 12;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 50;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            half_sectionA(600);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 6: // ターン調整 700mm/s

            printf("Mode 1-6.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 2000; // 5000
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 12700;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 41;
            val_offset_in = 680;
            angle_turn_90 = 85;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            half_sectionA(700);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 7: // ターン調整 900mm/s

            printf("Mode 1-7.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 4500;
            acceleration_straight_dash = 2000; // 5000
            // ターン
            velocity_turn90 = 900;
            alpha_turn90 = 22100;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 47;
            val_offset_in = 680;
            angle_turn_90 = 79;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            drive_fan(130);

            led_flash(20);

            velocity_interrupt = 0;

            half_sectionA(900);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);
            drive_fan(0);
            led_flash(5);
            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;
        }
    }
}
