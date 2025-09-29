/*
 * mode6.c
 *
 *  Created on: Aug 17, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"

void mode6() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //

            printf("Mode 6-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 15000;
            velocity_straight = 2000;

            // ターン
            velocity_turn90 = 1400;
            alpha_turn90 = 50000;
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

            drive_fan(400);

            led_flash(20);

            half_sectionA(1400);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 1: //

            printf("Mode 6-1 Large Turn 90deg.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 15000;
            velocity_straight = 2000;

            // 90°大回りターン
            velocity_l_turn_90 = 2000;
            alpha_l_turn_90 = 19000;
            angle_l_turn_90 = 88.0;
            dist_l_turn_out_90 = 52;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(400);

            led_flash(20);

            half_sectionA(2000);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 2:

            printf("Mode 6-2 Large Turn 180deg.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 15000;
            velocity_straight = 2000;

            // 180°大回りターン
            velocity_l_turn_180 = 2000;
            alpha_l_turn_180 = 20000;
            angle_l_turn_180 = 177;
            dist_l_turn_out_180 = 50;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(400);

            led_flash(20);

            half_sectionA(2000);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 3:
            printf("Mode 6-3.\n");
            run_shortest(6, 3);
            break;

        case 4:
            run_shortest(6, 4);
            break;

        case 5:
            run_shortest(6, 5);
            break;

        case 6:
            run_shortest(6, 6);
            break;

        case 7:
            run_shortest(6, 7);
            break;

        case 8:

            // 経路の重み
            straight_weight = 3; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunCaseParams_t *pc = &shortestRunCaseParamsMode6[3];
                const ShortestRunModeParams_t *pm = &shortestRunModeParams6;
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
                val_offset_in              = pm->val_offset_in;
                angle_turn_90              = pm->angle_turn_90;
                velocity_l_turn_90         = pm->velocity_l_turn_90;
                alpha_l_turn_90            = pm->alpha_l_turn_90;
                angle_l_turn_90            = pm->angle_l_turn_90;
                dist_l_turn_out_90         = pm->dist_l_turn_out_90;
                velocity_l_turn_180        = pm->velocity_l_turn_180;
                alpha_l_turn_180           = pm->alpha_l_turn_180;
                angle_l_turn_180           = pm->angle_l_turn_180;
                dist_l_turn_out_180        = pm->dist_l_turn_out_180;
                // 壁制御（caseごと）
                kp_wall                    = pc->kp_wall;
            }
            // 壁制御とケツ当て
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(550);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 9:

            // 経路の重み
            straight_weight = 1; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 28000;
            velocity_straight = 5000;
            // ターン
            velocity_turn90 = 1400;
            alpha_turn90 = 50000;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 58;
            val_offset_in = 1050;
            angle_turn_90 = 80.5;
            // 90°大回りターン
            velocity_l_turn_90 = 2000;
            alpha_l_turn_90 = 19200;
            angle_l_turn_90 = 89.5;
            dist_l_turn_in_90 = 0;
            dist_l_turn_out_90 = 67;
            // 180°大回りターン
            velocity_l_turn_180 = 2000;
            alpha_l_turn_180 = 20500;
            angle_l_turn_180 = 179;
            dist_l_turn_in_180 = 0;
            dist_l_turn_out_180 = 138;
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

            drive_fan(550);

            run();

            drive_fan(0);

            led_wait();

            break;
        }
    }
}
