/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"

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
            acceleration_straight = 2722;
            acceleration_straight_dash = 5000; // 5000
            velocity_straight = 1000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 1170;
            angle_turn_90 = 88.5;
            // 壁切れ後の距離
            dist_wall_end = 12;
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
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 8000; // 5000
            velocity_straight = 1000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 180°大回りターン
            velocity_l_turn_180 = 850;
            alpha_l_turn_180 = 3485;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 50;
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

        case 5:
            printf("Mode 2-5.\n");

            // 経路の重み
            straight_weight = 2; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 5000; // 5000
            velocity_straight = 3000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 180°大回りターン
            velocity_l_turn_180 = 850;
            alpha_l_turn_180 = 3485;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 45;
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

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 7000; // 5000
            velocity_straight = 3500;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 180°大回りターン
            velocity_l_turn_180 = 850;
            alpha_l_turn_180 = 3485;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 45;
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

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 9000; // 5000
            velocity_straight = 3500;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 180°大回りターン
            velocity_l_turn_180 = 850;
            alpha_l_turn_180 = 3485;
            angle_l_turn_180 = 179;
            dist_l_turn_out_180 = 45;
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
