/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"

void mode4() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: // 通常ターンの調整

            printf("Mode 4-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // ターン
            velocity_turn90 = 1000;
            alpha_turn90 = 22600;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
            val_offset_in = 680;
            angle_turn_90 = 88;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1000);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 1: // 90deg大回りターンの調整

            printf("Mode 4-1 Large Turn 90deg.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 90°大回りターン
            velocity_l_turn_90 = 1300;
            alpha_l_turn_90 = 7150;
            angle_l_turn_90 = 88;
            dist_l_turn_out_90 = 45;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1300);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 2: // 180deg大回りターンの調整

            printf("Mode 4-2 Large Turn 180deg.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 180°大回りターン
            velocity_l_turn_180 = 1300;
            alpha_l_turn_180 = 7300;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 40;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1300);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 3:
            printf("Mode 4-3.\n");
            run_shortest(4, 3);
            break;

        case 4:
            run_shortest(4, 4);
            break;

        case 5:
            run_shortest(4, 5);
            break;

        case 6:
            run_shortest(4, 6);
            break;

        case 7:
            run_shortest(4, 7);
            break;




        }
    }
}
