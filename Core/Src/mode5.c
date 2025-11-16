/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"

void mode5() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //

            printf("Mode 5-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 8000;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // ターン
            velocity_turn90 = 1200;
            alpha_turn90 = 30800;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
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

            drive_fan(300);

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

            run();

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

            run();

            drive_fan(0);

            led_wait();

            break;
        }
    }
}
