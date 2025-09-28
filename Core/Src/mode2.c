/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"

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

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(0);

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 2000; // 5000
            velocity_straight = 1500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 9000;
            acceleration_turn = 0;
            dist_offset_in = 8;
            dist_offset_out = 15.5; // 32
            val_offset_in = 1650;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            led_flash(3);

            run();

            break;

        case 4:
            printf("Mode 2-4.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode2[0];
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
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
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

            drive_fan(0);

            led_flash(5);

            run();

            drive_fan(0);

            MF.FLAG.RUNNING = 0;

            break;

        case 5:
            printf("Mode 2-5.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode2[1];
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
            // 壁切れ後の距離
            dist_wall_end = 12;
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

            run();

            MF.FLAG.RUNNING = 0;

            break;

        case 6:

            printf("Mode 2-6.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode2[2];
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
            // 壁切れ後の距離
            dist_wall_end = 12;
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

            run();

            MF.FLAG.RUNNING = 0;

            break;

        case 7:

            printf("Mode 2-7.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode2[3];
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
            // 壁切れ後の距離
            dist_wall_end = 12;
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

            run();

            MF.FLAG.RUNNING = 0;

            break;
        }
    }
}
