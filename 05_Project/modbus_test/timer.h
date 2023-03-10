/*
 * timer.h
 *
 *  Created on: Jun 27, 2022
 *      Author: Dimitar Lilov
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "device.h"

//
// Function Prototypes
//
void timer0_init(void);
void timer1_init(void);
void timer2_init(void);

#define timer0_get() CpuTimer0Regs.TIM.all
#define timer1_get() CpuTimer1Regs.TIM.all
#define timer2_get() CpuTimer2Regs.TIM.all

#define timer_init() timer2_init();
#define timer_get() timer2_get();

#endif /* TIMER_H_ */
//
// End of File
//




