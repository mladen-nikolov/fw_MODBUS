/* *****************************************************************************
 * File:   main.c
 * Author: XX
 *
 * Created on 2023 03 10
 *
 * Description: ...
 *
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include <string.h>
#include <stdint.h>

/* C2000 Ware Device support Include (Like MotorWare) */
#include "DSP28x_Project.h"

/* simple modbus_lib */
#include "modbus.h"

/* local includes */
#include "config.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */

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
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

uint32_t holding_registers_data[64];

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */

void holding_register_addresses_init(void)
{
    uint16_t index;
    uint32_t initial_data = 0x12345678;
    for(index = 0; index < 64; index++)
    {
        holding_registers_data[index] = initial_data++;

        /* simple modbus_lib */
        modbus_holding_register_address_set(index << 1, (uint32_t)&holding_registers_data[index]);
    }
}


void main(void)
{
    #if CONFIG_USE_DEBUG_SEND
    uint16_t u16WaitSeconds = 0;
    char pSeconds[3+1];
    pSeconds[0] = '\r';
    pSeconds[3] = 0;
    #endif

    // Initialize System Control: F2806x_SysCtrl.c
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    // Disable CPU interrupts
    DINT;

    // Init PIE control: F2806x_PieCtrl.c
    // The default state is all PIE interrupts disabled and flags cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table: F2806x_PieVect.c F2806x_DefaultIsr.c
    // This will populate the entire table, even if the interrupt not used
    InitPieVectTable();

    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: InitFlash();
    // For RamfuncsLoadStart, RamfuncsLoadSize, RamfuncsRunStart see .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();

    /* simple modbus_lib */
    modbus_init();

    // Init Registers in Modbus library
    holding_register_addresses_init();

    // Enable global Interrupts and higher priority real-time debug events
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    for(;;)
    {
        /* simple modbus_lib */
        modbus_uart_read_fast_loop_process();   /* this must be called frequent in order not to miss received bytes and overflow the hardware FIFO (4 levels Deep for 2806x) */

        modbus_main_loop_process();             /* can be called in main loop, but still keep in mind that the modbus timeouts are checked here. must be called at least (1.5 times / twice) faster than MODBUS_TIMEOUT_XXXX_MS definitions */

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
