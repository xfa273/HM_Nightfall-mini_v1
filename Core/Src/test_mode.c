/*
 * test_mode.c
 *
 *  Created on: Dec 23, 2023
 *      Author: yuho-
 */

#include "global.h"

void test_mode() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:

            printf("Test Mode 0.\n");

            break;

        case 1:

            printf("Test Mode 1 IMU Check.\n");

            drive_variable_reset();
            IMU_GetOffset();

            drive_fan(250);

            while (1) {

                printf("omega: %.3f [deg/s] ,  ", real_omega);

                printf("angle: %.3f [deg] ,  ", real_angle);

                printf("accl_x: %.3f ,  ", accel_x_true);

                printf("accl_y: %.3f\n", accel_y_true);

                HAL_Delay(100);
            }

            break;

        case 2:
            printf("Test Mode 2 Encoder Check.\n");

            drive_variable_reset();

            while (1) {

                printf("speed L R: %.3f %.3f [mm/s] ,  ", encoder_speed_l,
                       encoder_speed_r);
                printf("distance: %.3f [mm]\n",
                       (encoder_distance_l + encoder_distance_r) * 0.5);

                HAL_Delay(100);
            }

            break;
        case 3:
            printf("Test Mode 3 Sensor AD Value Check.\n");

            while (1) {
                printf("R: %d, L: %d, FR: %d, FL: %d, BAT: %d\n", ad_r, ad_l,
                       ad_fr, ad_fl, ad_bat);

                HAL_Delay(300);
            }
            break;

        case 4:
            printf("Test Mode 4 Enkai-Gei.\n");

            led_flash(10);

            IMU_GetOffset();
            drive_enable_motor();

            drive_variable_reset();

            // 回転角度カウントをリセット
            real_angle = 0;
            IMU_angle = 0;
            target_angle = 0;

            // 走行距離カウントをリセット
            real_distance = 0;
            encoder_distance_r = 0;
            encoder_distance_l = 0;

            led_write(1, 1);

            drive_start();

            drive_fan(250);

            break;
        case 5:
            printf("Test Mode 5.\n");

            load_map_from_eeprom();

            uint8_t fixedMap[MAZE_SIZE][MAZE_SIZE];

            // 迷路の初期化
            initializeMaze(fixedMap);

            // 下位4ビットを新しい配列にコピーし、表示する
            for (int i = 0; i < MAZE_SIZE; i++) {
                for (int j = 0; j < MAZE_SIZE; j++) {

                    // 下位4ビットを抽出し、新しい配列に格納
                    fixedMap[i][j] = (map[i][j] >> 4) & 0xF;
                    // 下位4ビットを2進数で表示
                    for (int k = 3; k >= 0; k--) {
                        printf("%d", (fixedMap[i][j] >> k) & 0x1);
                    }
                    printf(" ");
                }
                printf("\n");
            }

            reverseArrayYAxis(fixedMap);

            // 手動で設定した壁情報をセット
            // setMazeWalls(manualWalls);
            setMazeWalls(fixedMap);

            // 壁情報の矛盾を修正
            correctWallInconsistencies();

            // 最短経路を見つける
            Node start = {START_X, START_Y}; // スタート地点
            Node goal = {GOAL_X, GOAL_Y};    // ゴール地点

            dijkstra(start, goal);
            for (int i = 0; i < 256 && path[i] != 0; i++) {
                printf("%d ", path[i]);
            }
            printf("\n");

            simplifyPath();
            for (int i = 0; i < 256 && path[i] != 0; i++) {
                printf("%d ", path[i]);
            }
            printf("\n");

            convertLTurn();
            for (int i = 0; i < 256 && path[i] != 0; i++) {
                printf("%d ", path[i]);
            }
            printf("\n");

            // 迷路の表示
            printMaze();

            run();

            break;
        case 6:

            printf("Test Mode 6 Circuit.\n");

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 10000; // 5000
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
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(10, velocity_straight, 0);
            run_straight(8, velocity_straight, 0);
            run_straight(10, velocity_l_turn_90, 0);

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90();

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90();

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90();

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90();

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90();

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90();

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90();

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 8回目のターン
            l_turn_R90();

            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 7:

            printf("Test Mode 7 Circuit.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 28000;
            velocity_straight = 5000;
            // 90°大回りターン
            velocity_l_turn_90 = 2200;
            alpha_l_turn_90 = 28500;
            angle_l_turn_90 = 85.0;
            dist_l_turn_out_90 = 101;
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
            drive_fan(800);
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90();

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90();

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90();

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90();

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90();

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90();

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90();

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 8回目のターン
            l_turn_R90();

            half_sectionD(0);

            drive_fan(0);

            led_flash(5);
            drive_stop();

            break;

        case 8:

            printf("Test Mode 8 Circuit.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 30000;
            velocity_straight = 5200;
            // 90°大回りターン
            velocity_l_turn_90 = 2200;
            alpha_l_turn_90 = 28500;
            angle_l_turn_90 = 85.0;
            dist_l_turn_out_90 = 101;
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
            drive_fan(800);
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90();

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90();

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90();

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90();

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90();

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90();

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90();

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 8回目のターン
            l_turn_R90();

            half_sectionD(0);

            drive_fan(0);

            led_flash(5);
            drive_stop();

            break;

        case 9:

            printf("Test Mode 9 Circuit.\n");

            // 直線
            acceleration_straight = 10888.9;
            acceleration_straight_dash = 32000;
            velocity_straight = 5400;
            // 90°大回りターン
            velocity_l_turn_90 = 2200;
            alpha_l_turn_90 = 28500;
            angle_l_turn_90 = 85.0;
            dist_l_turn_out_90 = 101;
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
            drive_fan(800);
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90();

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90();

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90();

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90();

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90();

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90();

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90();

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            half_sectionD(0);

            drive_fan(0);

            led_flash(5);
            drive_stop();

            break;
        }
    }
}
