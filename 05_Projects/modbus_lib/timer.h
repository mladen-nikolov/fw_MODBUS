/* *****************************************************************************
 * File:   timer.h
 * Author: XX
 *
 * Created on 2023 03 10
 *
 * Description: ...
 *
 **************************************************************************** */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "device.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define timer_init() timer2_init();
#define timer_get() timer2_get();

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macro
 **************************************************************************** */
#define timer0_get() CpuTimer0Regs.TIM.all
#define timer1_get() CpuTimer1Regs.TIM.all
#define timer2_get() CpuTimer2Regs.TIM.all

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
void timer0_init(void);
void timer1_init(void);
void timer2_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */



