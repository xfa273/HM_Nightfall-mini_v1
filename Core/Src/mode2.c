/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"

void mode2() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //  通常ターンの調整

            printf("Mode 2-0 Normal Turn.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000

            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 29;
            val_offset_in = 680;
            angle_turn_90 = 88.5;

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

        case 1: // 90deg大回りターンの調整

            printf("Mode 2-1 Large Turn 90deg.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000

            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 2970;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 21;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            half_sectionA(850);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 2: // 180deg大回りターンの調整

            printf("Mode 2-2 Large Turn 180deg.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000

            // 180°大回りターン
            velocity_l_turn_180 = 850;
            alpha_l_turn_180 = 3485;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 32;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            half_sectionA(850);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 3:
            printf("Mode 2-3.\n");
            run_shortest(2, 3);
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
