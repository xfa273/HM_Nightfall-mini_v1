/*
 * mode7.c
 *
 *  Created on: Aug 17, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"

void mode7() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //

            printf("Mode 6-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // ターン
            velocity_turn90 = 1400;
            alpha_turn90 = 60000;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
            val_offset_in = 1100;
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

            drive_fan(600);

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
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // 90°大回りターン
            velocity_l_turn_90 = 2200;
            alpha_l_turn_90 = 27500;
            angle_l_turn_90 = 85.0;
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

            drive_fan(600);

            led_flash(20);

            half_sectionA(2200);
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
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // 180°大回りターン
            velocity_l_turn_180 = 2200;
            alpha_l_turn_180 = 23000;
            angle_l_turn_180 = 176.8;
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

            drive_fan(600);

            led_flash(20);

            half_sectionA(2200);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 3:
            printf("Mode 6-3.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(0);

            // 直線
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;
            // ターン
            velocity_turn90 = 1400;
            alpha_turn90 = 50000;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 58;
            val_offset_in = 1050;
            angle_turn_90 = 80.5;
            // 壁制御とケツ当て
            kp_wall = 0.13;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(3);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(600);

            led_flash(10);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 4:
            printf("Mode 6-4.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode7[0];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
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

            drive_fan(700);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 5:
            printf("Mode 6-5.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode7[1];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
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

            drive_fan(750);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 6:

            printf("Mode 6-6.\n");

            // 経路の重み
            straight_weight = 3; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode7[2];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
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

            drive_fan(750);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 7:

            printf("Mode 6-7.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode7[3];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;

                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
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

            drive_fan(750);

            run();

            drive_fan(0);

            led_wait();

            break;
        }
    }
}
