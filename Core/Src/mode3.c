/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"

void mode3() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: // 通常ターンの調整

            printf("Mode 3-0 Normal Turn.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 5000; // 5000
            velocity_straight = 4000;

            // ターン
            velocity_turn90 = 800;
            alpha_turn90 = 15550;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 30;
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

            drive_fan(100);

            led_flash(20);

            half_sectionA(800);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);

            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 1: // 90deg大回りターンの調整

            printf("Mode 3-1 Large Turn 90deg.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 0; // 5000
            velocity_straight = 0;

            // 90°大回りターン
            velocity_l_turn_90 = 1000;
            alpha_l_turn_90 = 4190;
            angle_l_turn_90 = 88.7;
            dist_l_turn_out_90 = 22;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(100);

            led_flash(20);

            half_sectionA(1000);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);

            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 2: // 180deg大回りターンの調整

            printf("Mode 3-2 Large Turn 180deg.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 0; // 5000
            velocity_straight = 0;

            // 180°大回りターン
            velocity_l_turn_180 = 1000;
            alpha_l_turn_180 = 4680;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 38;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(100);

            led_flash(20);

            half_sectionA(1000);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);

            drive_stop();

            MF.FLAG.RUNNING = 0;

            break;

        case 3:
            printf("Mode 3-3.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(0);

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 10000; // 5000
            velocity_straight = 3500;
            // ターン
            velocity_turn90 = 800;
            alpha_turn90 = 15550;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 43;
            val_offset_in = 710;
            angle_turn_90 = 88.5;
            // 壁制御とケツ当て
            kp_wall = 0.06;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            MF.FLAG.SCND = 1;
            MF.FLAG.RETURN = 0;

            led_flash(5);

            get_base();

            drive_fan(200);

            led_flash(3);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;

        case 4:
            printf("Mode 3-4.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            // パラメータを設定
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode3[0];
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
                dist_l_turn_in_90          = p->dist_l_turn_in_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_in_180         = p->dist_l_turn_in_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }

            // 壁制御とケツ当て
            kp_wall = 0.2;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            MF.FLAG.SCND = 1;
            MF.FLAG.RETURN = 0;

            led_flash(5);

            get_base();

            drive_fan(200);

            led_flash(3);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;

        case 5:
            printf("Mode 3-5.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode3[1];
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
                dist_l_turn_in_90          = p->dist_l_turn_in_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_in_180         = p->dist_l_turn_in_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.2;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            MF.FLAG.SCND = 1;
            MF.FLAG.RETURN = 0;

            led_flash(5);

            get_base();

            drive_fan(200);

            led_flash(3);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;

        case 6:

            printf("Mode 3-6.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode3[2];
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
                dist_l_turn_in_90          = p->dist_l_turn_in_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_in_180         = p->dist_l_turn_in_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.2;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            MF.FLAG.SCND = 1;
            MF.FLAG.RETURN = 0;

            led_flash(5);

            get_base();

            drive_fan(200);

            led_flash(3);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;

        case 7:

            printf("Mode 3-7.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode3[3];
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
                dist_l_turn_in_90          = p->dist_l_turn_in_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_in_180         = p->dist_l_turn_in_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.2;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            MF.FLAG.SCND = 1;
            MF.FLAG.RETURN = 0;

            led_flash(5);

            get_base();

            drive_fan(200);

            led_flash(3);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;
        }
    }
}
