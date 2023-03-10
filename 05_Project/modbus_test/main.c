/*
 * main.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Dimitar Lilov
 */


//
// Included Files
//
#include "DSP28x_Project.h"     // Device Header file and Examples Include File
#include <string.h>
#include <stdint.h>

#include "modbus.h"

#include "config.h"
//#include "timer.h"
//#include "scia.h"
//#include "device.h"
//#include "gpio.h"


//
// These are defined by the linker (see F2808.cmd)
//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;




uint32_t holding_registers_data[64];

void holding_register_addresses_init(void)
{
    uint16_t index;
    uint32_t initial_data = 0x12345678;
    for(index = 0; index < 64; index++)
    {
        holding_registers_data[index] = initial_data++;
        modbus_holding_register_address_set(index << 1, (uint32_t)&holding_registers_data[index]);
    }

}


//
// Main
//
void main(void)
{

    #if CONFIG_USE_DEBUG_SEND
    uint16_t u16WaitSeconds = 0;
    //uint32_t u32TicksTimerAlive = 0;
    char pSeconds[3+1];
    pSeconds[0] = '\r';
    pSeconds[3] = 0;
    #endif


    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();


    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: InitFlash();
    // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the F2808.cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

    //
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //
    InitFlash();

    modbus_init();

    holding_register_addresses_init();


    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    for(;;)
    {
        modbus_uart_read_fast_loop_process();
        modbus_main_loop_process();

        #if CONFIG_USE_DEBUG_SEND
        #define DELAY 500000L
        DELAY_US(DELAY);
        u16WaitSeconds++;
        pSeconds[1] = ((u16WaitSeconds / 10) > 0) ? (u16WaitSeconds / 10) + 0x30 : ' ';
        pSeconds[2] = (u16WaitSeconds % 10) + 0x30;
        scia_SendString(pSeconds);
        #endif
    }
}
