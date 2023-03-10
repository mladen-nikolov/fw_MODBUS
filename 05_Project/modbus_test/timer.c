/*
 * timer.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Dimitar Lilov
 */

//
// Included Files
//
#include "device.h"

//
// Globals
//


void timer0_init(void)
{
    //
    // CPU Timer 0
    //

    //
    // Initialize address pointers to respective timer registers
    //
    //CpuTimer0.RegsAddr = &CpuTimer0Regs;

    //
    // Initialize timer period to maximum
    //
    CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all = 0;

    //
    // Make sure timer is stopped
    //
    CpuTimer0Regs.TCR.bit.TSS = 1;

    //
    // Reload all counter register with period value
    //
    CpuTimer0Regs.TCR.bit.TRB = 1;

    //
    // Use write-only instruction to set TSS bit = 0
    //
    CpuTimer0Regs.TCR.all = 0x4000;
}

void timer1_init(void)
{
    //
    // CPU Timer 0
    //

    //
    // Initialize address pointers to respective timer registers
    //
    //CpuTimer1.RegsAddr = &CpuTimer1Regs;

    //
    // Initialize timer period to maximum
    //
    CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CpuTimer1Regs.TPR.all  = 0;
    CpuTimer1Regs.TPRH.all = 0;

    //
    // Make sure timer is stopped
    //
    CpuTimer1Regs.TCR.bit.TSS = 1;

    //
    // Reload all counter register with period value
    //
    CpuTimer1Regs.TCR.bit.TRB = 1;

    //
    // Use write-only instruction to set TSS bit = 0
    //
    CpuTimer1Regs.TCR.all = 0x4000;
}

void timer2_init(void)
{
    //
    // CPU Timer 0
    //

    //
    // Initialize address pointers to respective timer registers
    //
    //CpuTimer2.RegsAddr = &CpuTimer2Regs;

    //
    // Initialize timer period to maximum
    //
    CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CpuTimer2Regs.TPR.all  = 0;
    CpuTimer2Regs.TPRH.all = 0;

    //
    // Make sure timer is stopped
    //
    CpuTimer2Regs.TCR.bit.TSS = 1;

    //
    // Reload all counter register with period value
    //
    CpuTimer2Regs.TCR.bit.TRB = 1;

    //
    // Use write-only instruction to set TSS bit = 0
    //
    CpuTimer2Regs.TCR.all = 0x4000;
}



//
// End of File
//





