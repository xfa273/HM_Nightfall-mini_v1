/*
 * drive.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "main.h"
#include "params.h"
#include "drive.h"
#include "sensor.h"
#include "interrupt.h"
#include "logging.h"
#include <math.h>

/*==========================================================
    走行系 上位関数
==========================================================*/
/*
    マウスフラグ(MF)
      6Bit:デフォルトインターバルフラグ
      5Bit:減速フラグ
      4Bit:加速フラグ
      3Bit:制御フラグ
      1Bit:二次走行フラグ
*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// first_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void first_sectionA(void) {
    float speed_out;

    speed_out =
        sqrt(speed_now * speed_now + 2 * acceleration_straight * DIST_HALF_SEC);

    MF.FLAG.CTRL = 1;
    driveA(DIST_FIRST_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && !val) {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight_dash * DIST_HALF_SEC);
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * DIST_HALF_SEC);
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionAD
// 斜め半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionAD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && !val) {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * DIST_D_HALF_SEC);
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(DIST_D_HALF_SEC, speed_now, speed_out, dist_wall_end + 90);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight_dash * DIST_HALF_SEC);
        } else if (!val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight * DIST_HALF_SEC);
        } else {
            speed_out = 0;
        }
    }

    MF.FLAG.CTRL = 1;

    driveA(DIST_HALF_SEC, speed_now, speed_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (!val) {
        velocity_interrupt = 0;
    }

    // get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionDD
// 斜め半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionDD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else if (!val) {
            speed_out = 0;
        } else {
            speed_out = 0;
        }
    }

    MF.FLAG.CTRL = 1;

    driveA(DIST_D_HALF_SEC, speed_now, speed_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (!val) {
        velocity_interrupt = 0;
    }

    // get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionA
// 1区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void) {
    float speed_out;
    if (MF.FLAG.SCND || known_straight) {
        speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight_dash * DIST_HALF_SEC * 2);
    } else {
        speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight * DIST_HALF_SEC * 2);
    }

    MF.FLAG.CTRL = 1;
    // kp_wall = kp_wall * 2;
    driveA(DIST_HALF_SEC * 2, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    // kp_wall = kp_wall * 1 / 2;
    speed_now = speed_out;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionD
//  1区間減速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void) {
    float speed_out;
    if (MF.FLAG.SCND || acceled) {
        speed_out = sqrt(speed_now * speed_now -
                         2 * acceleration_straight_dash * DIST_HALF_SEC * 2);
    } else {
        speed_out = sqrt(speed_now * speed_now -
                         2 * acceleration_straight * DIST_HALF_SEC * 2);
    }
    MF.FLAG.CTRL = 1;

    if (ad_fl > WALL_BASE_FL || ad_fr > WALL_BASE_FR) {
        MF.FLAG.F_WALL_STOP = 1;
    }

    driveA(DIST_HALF_SEC * 2, speed_now, speed_out, 0);

    MF.FLAG.F_WALL_STOP = 0;

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_section
// 1区画分進んで停止する。1区画走行用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void) {}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(uint8_t CTRL) {
    (void)CTRL;
    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC * 2, speed_now, speed_now, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_now;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionU(void) {
    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC, speed_now, speed_now, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_now;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_straight
// 指定区画を指定速度で走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_straight(float section, float spd_out, float dist_wallend) {

    MF.FLAG.CTRL = 1;

    if (dist_wallend > 0 && section > 1 && spd_out > 100) {
        driveA(DIST_HALF_SEC * section, speed_now, spd_out, dist_wallend);
    } else {
        driveA(DIST_HALF_SEC * section, speed_now, spd_out, 0);
    }

    MF.FLAG.CTRL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void) {
    driveR(ANGLE_ROTATE_90_R);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void) {
    driveR(-ANGLE_ROTATE_90_L);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void) { driveR(ANGLE_ROTATE_90_R * 2); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R90
// スラロームで右に90度旋回前進する
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R90(uint8_t fwall) {
    MF.FLAG.SLALOM_R = 1;
    MF.FLAG.CTRL = 1;

    if (fwall) {
        if (MF.FLAG.F_WALL) {
            driveFWall(dist_offset_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_offset_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_offset_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSR(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 1;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L90
// スラロームで左に90度旋回前進する
// 引数：引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L90(uint8_t fwall) {
    MF.FLAG.SLALOM_L = 1;
    MF.FLAG.CTRL = 1;

    if (fwall) {

        if (MF.FLAG.F_WALL) {
            driveFWall(dist_offset_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_offset_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_offset_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSL(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 1;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R90
// 90度大回り右旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.CTRL = 1;
    driveA(dist_l_turn_out_90, speed_now, velocity_l_turn_90, 0);
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L90
// 90度大回り左旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.CTRL = 1;
    driveA(dist_l_turn_out_90, speed_now, velocity_l_turn_90, 0);
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R180
// 180度大回り右旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R180(uint8_t fwall) {
    (void)fwall;
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.CTRL = 1;
    driveA(dist_l_turn_out_180, speed_now, velocity_l_turn_180, 0);
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L180
// 180度大回り左旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L180(uint8_t fwall) {
    (void)fwall;
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.CTRL = 1;
    driveA(dist_l_turn_out_180, speed_now, velocity_l_turn_180, 0);
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_In(void) {

    MF.FLAG.CTRL = 0;
    driveSR(angle_turn45in, alpha_turn45in);
    driveA(dist_turn45in, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSR(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_In(void) {

    MF.FLAG.CTRL = 0;
    driveSL(angle_turn45in, alpha_turn45in);
    driveA(dist_turn45in, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSL(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_RV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_RV90(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSR(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_LV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_LV90(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSL(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_In(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSR(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSR(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_In(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSL(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSL(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_diagonal
// 斜め直進
// 引数：区画数，終端速度
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_diagonal(float section, float spd_out) {
    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 1;

    driveA(DIST_D_HALF_SEC * section, speed_now, spd_out, 0);

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_R90
// スラロームで右に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_R90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_R90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_L90
// スラロームで左に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_L90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_L90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：sw …… 0以外ならget_base()する
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(void) {

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ後進
    velocity_interrupt = -130;
    drive_start();
    HAL_Delay(400);
    drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ前進
    driveA(DIST_SET_POSITION * 0.5, 0, 200, 0);
    driveA(DIST_SET_POSITION * 0.5, 200, 0, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// match_position
// 壁との距離を基準値に合わせる
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void match_position(uint16_t target_value) {

    while (abs(target_value - (ad_fr + ad_fl)) > 10) {

        // 未使用変数を削除
    }
}

/*==========================================================
    走行系 基幹関数
==========================================================*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveA
// 指定パルス分加速走行する
// 引数1：dist …… 走行する距離[mm]
// 引数2: spd_in …… 初速度[mm/sec]
// 引数3: spd_out …… 到達速度[mm/sec]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(float dist, float spd_in, float spd_out, float dist_wallend) {

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    if (dist_wallend > 0) {
        // MF.FLAG.WALL_END = 1;
    } else {
        // MF.FLAG.WALL_END = 0;
    }

    // 実際の距離が目標距離になるまで走行
    if (acceleration_interrupt > 0) {

        while (real_distance < dist)
            ;

    } else if (acceleration_interrupt <= 0) {

        if (MF.FLAG.F_WALL_STOP) {
            while (real_distance < dist && velocity_interrupt > 0 &&
                   (ad_fl + ad_fr) < thr_f_wall) {
            };
        } else if (MF.FLAG.WALL_END) {
            if (dist > 90) {
                while (real_distance < dist - 90) {
                };
            }
            while (real_distance < (dist + 10) && velocity_interrupt > 0 &&
                   !MF.FLAG.R_WALL_END && !MF.FLAG.L_WALL_END) {
            };
            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                buzzer_interrupt(300);
                // 走行距離カウントをリセット
                real_distance = 0;
                encoder_distance_r = 0;
                encoder_distance_l = 0;
                while (real_distance < dist_wallend) {
                };
            }

        } else {
            while (real_distance < dist && velocity_interrupt > 0) {
            };
        }
    }

    MF.FLAG.WALL_END = 0;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveR
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行する距離[mm]
// 引数2：spd …… 速度[mm/s]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveR(float angle) {

    // 角加速度を設定
    if (angle >= 0) {
        alpha_interrupt = ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = -ALPHA_ROTATE_90;
    }

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    velocity_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    if (angle >= 0) {
        while (real_angle > -angle * 0.333 && !MF.FLAG.FAILED)
            ;
    } else {
        while (real_angle < -angle * 0.333 && !MF.FLAG.FAILED)
            ;
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    if (angle >= 0) {
        while (real_angle > -angle * 0.666 && !MF.FLAG.FAILED)
            ;
    } else {
        while (real_angle < -angle * 0.666 && !MF.FLAG.FAILED)
            ;
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    if (angle >= 0) {
        alpha_interrupt = -ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = +ALPHA_ROTATE_90;
    };
    if (angle >= 0) {
        while (real_angle > -angle && !MF.FLAG.FAILED)
            ;
    } else {
        while (real_angle < -angle && !MF.FLAG.FAILED)
            ;
    }

    alpha_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveC
// 指定パルス分デフォルトインターバルで走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(int dist) {
    (void)dist;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveM
// センサ値が基準値になるまで前進or後進する
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveM(uint16_t sens_tgt) {
    (void)sens_tgt;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSR
// 指定速度でスラロームで右旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSR(float angle_turn, float alpha_turn) {

    // printf("%f\n", alpha_interrupt);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    drive_start();

    // 角加速度と並進加速度を設定
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle > -angle_turn * 0.333 && !MF.FLAG.FAILED)
        ;

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;

    while (real_angle > -angle_turn * 0.666 && !MF.FLAG.FAILED)
        ;

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle > -angle_turn && !MF.FLAG.FAILED)
        ;

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSL
// 指定速度でスラロームで左旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSL(float angle_turn, float alpha_turn) {

    // printf("%f\n", alpha_interrupt);
    // printf("%f\n", alpha_interrupt);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;

    // 角加速度と並進加速度を設定
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle < angle_turn * 0.333 && !MF.FLAG.FAILED)
        ;

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;
    while (real_angle < angle_turn * 0.666 && !MF.FLAG.FAILED)
        ;

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle < angle_turn && !MF.FLAG.FAILED)
        ;

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
}

void driveFWall(float dist, float spd_in, float spd_out) {

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    // 実際の距離が目標距離になるか前壁距離に達するまで走行
    if (MF.FLAG.SLALOM_R) {
        while ((ad_fr + ad_fl) < val_offset_in)
            ;
    } else if (MF.FLAG.SLALOM_L) {
        while ((ad_fr + ad_fl) < val_offset_in)
            ;
    }

    velocity_interrupt = spd_out;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

void driveWallEnd(float dist, float spd_in, float spd_out) {

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    // 実際の距離が目標距離になるか壁切れ検知まで走行
    if (r_wall && l_wall) {
        // 左右壁がある

        while (real_distance < dist && (r_wall && l_wall))
            ;

        if (real_distance < (dist - 5)) {
            // 指定の後距離を走行
            dist_wall_end += real_distance;
            while (real_distance < dist_wall_end)
                ;
        }
    } else if (r_wall) {
        // 右壁だけある

        while (real_distance < dist && r_wall)
            ;

        if (real_distance < dist - 5) {
            // 指定の後距離を走行
            dist_wall_end += real_distance;
            while (real_distance < dist_wall_end)
                ;
        }
    } else if (r_wall) {
        // 左壁だけある

        while (real_distance < dist && l_wall)
            ;

        if (real_distance < dist - 5) {
            // 指定の後距離を走行
            dist_wall_end += real_distance;
            while (real_distance < dist_wall_end)
                ;
        }
    }

    velocity_interrupt = spd_out;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

void adjust_wallend(void) {}

/*==========================================================
    初期化関数・設定関数・その他関数
==========================================================*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_init
// 走行系の変数の初期化，モータードライバ関係のGPIO設定とPWM出力に使うタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void) {
    // エンコーダの読み取り開始
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    wall_end_count = 0;

    //====走行系の変数の初期化====

    //====マウスフラグの初期化===
    MF.FLAGS = 0; // フラグクリア
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_variable_reset
// 割込み内の走行系の変数の初期化
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_variable_reset(void) {

    // 速度
    acceleration_interrupt = 0;
    target_distance = 0;

    // 角度
    alpha_interrupt = 0;
    omega_interrupt = 0;
    target_angle = 0;

    // PIDの積算項（距離）
    distance_error = 0;
    previous_distance_error = 0;
    distance_error_error = 0;
    distance_integral = 0;

    // PIDの積算項（速度）
    velocity_error = 0;
    previous_velocity_error = 0;
    velocity_error_error = 0;
    velocity_integral = 0;

    // PIDの積算項（角度）
    angle_error = 0;
    previous_angle_error = 0;
    angle_error_error = 0;
    angle_integral = 0;

    // PIDの積算項（角速度）
    target_omega = 0;
    omega_error = 0;
    previous_omega_error = 0;
    omega_error_error = 0;
    omega_integral = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_enable_motor
// モータドライバの電源ON
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_enable_motor(void) {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_disable_motor
// モータードライバの電源OFF
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_disable_motor(void) {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_start
// 走行を開始する
// （pulse_l,pulse_rを0にリセットしてタイマを有効にする）
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_stop
// 走行を終了する
// （タイマを止めてタイマカウント値を0にリセットする）
// 引数1：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_set_dir
// 進行方向を設定する
// 引数1：d_dir …… どの方向に進行するか  0桁目で左，1桁目で右の方向設定
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_set_dir(uint8_t d_dir) {
    // printf("DIR\n");

    //====左モータ====
    switch (d_dir & 0x0f) { // 0~3ビット目を取り出す
    //----正回転----
    case 0x00: // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_FWD_L);
        // 左を前進方向に設定
        break;
    //----逆回転----
    case 0x01: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_BACK_L);
        // 左を後進方向に設定
        break;
    }
    //====右モータ====
    switch (d_dir & 0xf0) { // 4~7ビット目を取り出す
    case 0x00:              // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_FWD_R);
        // 右を前進方向に設定
        break;
    //----逆回転----
    case 0x10: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_BACK_R);
        // 右を後進方向に設定
        break;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_motor
// モータを回す
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_motor(void) {

    // 並進と回転の出力を合算
    out_r = out_translation - out_rotate;
    out_l = out_translation + out_rotate;

    // 回転のフェイルセーフ
    if (((out_r - out_l > FAIL_LR_ERROR || out_r - out_l < -FAIL_LR_ERROR)) &&
        MF.FLAG.RUNNING) {
        fail_count_lr++;
    } else {
        fail_count_lr = 0;
    }

    if (fail_count_lr > FAIL_COUNT_LR) {
        MF.FLAG.FAILED = 1;
        fail_count_lr = 0;
    }

    // 並進衝突のフェイルセーフ
    if ((IMU_acceleration < -FAIL_ACC || IMU_acceleration > FAIL_ACC) &&
        MF.FLAG.RUNNING) {
        fail_count_acc++;
    } else {
        fail_count_acc = 0;
    }

    if (fail_count_acc > FAIL_COUNT_ACC && MF.FLAG.RUNNING) {
        MF.FLAG.FAILED = 1;
        fail_count_acc = 0;
    }

    // 左右モータの回転方向の指定
    if (out_r >= 0 && out_l >= 0) {
        drive_set_dir(0x00);
    } else if (out_r >= 0 && out_l < 0) {
        drive_set_dir(0x01);
        out_l = -out_l;
    } else if (out_r < 0 && out_l >= 0) {
        drive_set_dir(0x10);
        out_r = -out_r;
    } else if (out_r < 0 && out_l < 0) {
        drive_set_dir(0x11);
        out_r = -out_r;
        out_l = -out_l;
    }

    // PWM出力
    if (MF.FLAG.FAILED) {
        // フェイルセーフ発動の場合，強制的に停止
        drive_enable_motor();
        drive_fan(0);

        buzzer_beep(1200);

    } else {
        //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, min(out_r, 1000));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, min(out_l, 1000));
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_fan
// ファンを回す
// 引数：fan_power (0~1000)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_fan(uint16_t fan_power) {

    if (fan_power > 0) {
        MF.FLAG.SUCTION = 1;
    } else {
        MF.FLAG.SUCTION = 0;
    }

    if (fan_power) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

        for (uint16_t i = 0; i < fan_power; i++) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            HAL_Delay(4);
        }
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// test_run
// テスト走行モード
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_run(void) {
    int mode = 0;
    drive_enable_motor();

    led_flash(8);

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:
            //----尻当て----
            printf("Mode 4-0 Set Position.\n");

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 11300;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 33.5;
            val_offset_in = 1500; // 1790
            angle_turn_90 = 90;
            // 90°大回りターン
            velocity_l_turn_90 = 700;
            alpha_l_turn_90 = 2030;
            angle_l_turn_90 = 90;
            dist_l_turn_out_90 = 34;
            // 180°大回りターン
            velocity_l_turn_180 = 700;
            alpha_l_turn_180 = 2000;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 45;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            set_position();

            led_flash(5);
            drive_stop();

            break;
        case 1:
            // ログ確認用 Mode2以降だと何故かログ取れない
            printf("Mode 6-1 Get Log.\n");

            // get_base();

            acceleration_straight = 600;
            acceleration_straight_dash = 0; // 5000
            velocity_turn90 = 300;
            alpha_turn90 = 14000;
            acceleration_turn = 0;
            dist_offset_in = 20;
            dist_offset_out = 34;
            val_offset_in = 1700; // 1790
            angle_turn_90 = 90;
            kp_wall = 0.0;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_start(HAL_GetTick());
            
            // half_sectionA(0);

            // turn_R90(0);
            rotate_180();

            // half_sectionD(0);
            log_stop();

            drive_stop();

            break;
        case 2:
            // 新しいロギングシステムでログを出力
            printf("Mode 4-2 - ロギングデータの出力\n");
            log_print_all();

            break;
        case 3:
            // 純粋なCSVデータのみを出力（可視化ツール貼り付け用）
            printf("Mode 4-3 - CSV形式ログ出力（可視化ツール用）\n");
            log_print_csv_only();

            break;
        case 4:
            //----直進----
            printf("Mode 4-4 straight 2 Sections.\n");

            // 直線
            acceleration_straight = 1000;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 5:
            //----右旋回----
            printf("Mode 4-5 Turn R90.\n");

            // 直線
            acceleration_straight = 444.44;
            acceleration_straight_dash = 444.44; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 11500;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 39;
            val_offset_in = 500;
            angle_turn_90 = 86.7;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(0);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;
        case 6:
            //--------
            printf("Mode 4-6 Rotate R90.\n");

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 11500;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 39;
            val_offset_in = 680;
            angle_turn_90 = 86.7;
            // 90°大回りターン
            velocity_l_turn_90 = 700;
            alpha_l_turn_90 = 2030;
            angle_l_turn_90 = 89.6;
            dist_l_turn_out_90 = 40;
            // 180°大回りターン
            velocity_l_turn_180 = 700;
            alpha_l_turn_180 = 2000;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 43;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            get_base();

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            first_sectionA();
            for (uint8_t i = 0; i < 7; i++) {
                half_sectionU();
            }
            turn_R90(1);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;
        case 7:

            printf("Mode 4-7.\n");

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 11500;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 39;
            val_offset_in = 680;
            angle_turn_90 = 86.7;
            // 90°大回りターン
            velocity_l_turn_90 = 900;
            alpha_l_turn_90 = 2970;
            angle_l_turn_90 = 88;
            dist_l_turn_out_90 = 27;
            // 180°大回りターン
            velocity_l_turn_180 = 1000;
            alpha_l_turn_180 = 2900;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 43;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(velocity_l_turn_90);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 8:

            printf("Mode 4-8.\n");

            // 直線
            acceleration_straight = 2722;
            acceleration_straight_dash = 3000; // 5000
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 11500;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 39;
            val_offset_in = 680;
            angle_turn_90 = 86.7;
            // 90°大回りターン
            velocity_l_turn_90 = 900;
            alpha_l_turn_90 = 2670;
            angle_l_turn_90 = 89.6;
            dist_l_turn_out_90 = 40;
            // 180°大回りターン
            velocity_l_turn_180 = 1000;
            alpha_l_turn_180 = 4330;
            angle_l_turn_180 = 176;
            dist_l_turn_out_180 = 42;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(velocity_l_turn_180);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;
        }
    }
    drive_disable_motor();
}

void reset_failed(void) {

    drive_stop();
    drive_variable_reset();

    for (int i = 0; i < 5; i++) {
        buzzer_beep(900);
    }

    MF.FLAG.FAILED = 0;
}
