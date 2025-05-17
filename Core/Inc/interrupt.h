/*
 * interrupt.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

void tim1_wait_us(uint32_t);

#endif /* INC_INTERRUPT_H_ */

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
volatile uint8_t ADC_task_counter; // ADCの振り分け用カウンタ

volatile float log_1[1000]; // ログ(1)
volatile float log_2[1000]; // ログ(2)
volatile float log_3[1000]; // ログ(3)
volatile float log_4[1000]; // ログ(4)
volatile float log_5[1000]; // ログ(5)
volatile float log_6[1000]; // ログ(6)
volatile float log_7[1000]; // ログ(7)
volatile float log_8[1000]; // ログ(8)
volatile uint16_t log_cnt;

#else // main.c以外からこのファイルが呼ばれている場合
extern volatile uint8_t ADC_task_counter; // ADCの振り分け用カウンタ

extern volatile float log_1[1000]; // ログ(1)
extern volatile float log_2[1000]; // ログ(2)
extern volatile float log_3[1000]; // ログ(3)
extern volatile float log_4[1000]; // ログ(4)
extern volatile float log_5[1000]; // ログ(5)
extern volatile float log_6[1000]; // ログ(6)
extern volatile float log_7[1000]; // ログ(7)
extern volatile float log_8[1000]; // ログ(8)
extern volatile uint16_t log_cnt;

#endif
