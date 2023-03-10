//###########################################################################
//
// FILE:	Example_28069Flash.c
//
// TITLE:	ePWM Timer Interrupt From Flash Example
//
//! \addtogroup f2806x_example_list
//! <h1>ePWM Timer Interrupt From Flash (flash_f28069)</h1>
//!
//! This example runs the ePWM interrupt example from flash. ePwm1 Interrupt 
//! will run from RAM and puts the flash into sleep mode. ePwm2 Interrupt 
//! will run from RAM and puts the flash into standby mode. ePWM3 Interrupt
//! will run from FLASH. All timers have the same period. The timers are 
//! started sync'ed. An interrupt is taken on a zero event for each ePWM 
//! timer.GPIO34 is toggled while in the background loop.
//! Note:
//!   - ePWM1: takes an interrupt every event
//!   - ePWM2: takes an interrupt every 2nd event
//!   - ePWM3: takes an interrupt every 3rd event 
//! Thus the Interrupt count for ePWM1, ePWM4-ePWM6 should be equal
//! The interrupt count for ePWM2 should be about half that of ePWM1
//! and the interrupt count for ePWM3 should be about 1/3 that of ePWM1
//! 
//! Follow these steps to run the program.
//!   - Build the project
//!   - Flash the .out file into the device.
//!   - Set the hardware jumpers to boot to Flash (put position 1 and 2 of 
//!     SW2 on control Card to ON position).
//!   - Use the included GEL file to load the project, symbols
//!     defined within the project and the variables into the watch
//!     window.
//! 
//! Steps that were taken to convert the ePWM example from RAM
//! to Flash execution: 
//! - Change the linker cmd file to reflect the flash memory map.
//! - Make sure any initialized sections are mapped to Flash.
//!   In SDFlash utility this can be checked by the View->Coff/Hex
//!   status utility. Any section marked as "load" should be
//!   allocated to Flash.
//! - Make sure there is a branch instruction from the entry to Flash
//!   at 0x3F7FF6 to the beginning of code execution. This example
//!   uses the DSP0x_CodeStartBranch.asm file to accomplish this.
//! - Set boot mode Jumpers to "boot to Flash"
//! - For best performance from the flash, modify the waitstates
//!   and enable the flash pipeline as shown in this example.
//!   Note: any code that manipulates the flash waitstate and pipeline
//!   control must be run from RAM. Thus these functions are located
//!   in their own memory section called ramfuncs.
//! 
//! \b Watch \b Variables \n
//!  - EPwm1TimerIntCount
//!  - EPwm2TimerIntCount
//!  - EPwm3TimerIntCount
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:
// Copyright (C) 2009-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "string.h"
#include <stdint.h>
//#include "array.h"
#include "led.h"
#include "timer.h"
#include "scia.h"
#include "device.h"
////
//// Defines that configure which ePWM timer interrupts are enabled at the
//// PIE level: 1 = enabled,  0 = disabled
////
//#define PWM1_INT_ENABLE  1
//#define PWM2_INT_ENABLE  1
//#define PWM3_INT_ENABLE  1
//
////
//// Defines that configure the period for each timer
////
//#define PWM1_TIMER_TBPRD   0x1FFF
//#define PWM2_TIMER_TBPRD   0x1FFF
//#define PWM3_TIMER_TBPRD   0x1FFF

//
// Defines that make this long enough so that we can see an LED toggle
//
#define DELAY 500000L

////
//// Functions that will be run from RAM need to be assigned to
//// a different section.  This section will then be mapped using
//// the linker cmd file.
////
//#pragma CODE_SECTION(epwm1_timer_isr, "ramfuncs");
//#pragma CODE_SECTION(epwm2_timer_isr, "ramfuncs");
//
////
//// Function Prototypes
////
//interrupt void epwm1_timer_isr(void);
//interrupt void epwm2_timer_isr(void);
//interrupt void epwm3_timer_isr(void);
//void InitEPwmTimer(void);
//
////
//// Globals
////
//Uint32  EPwm1TimerIntCount;
//Uint32  EPwm2TimerIntCount;
//Uint32  EPwm3TimerIntCount;
//Uint32  LoopCount;
//
//
// These are defined by the linker (see F2808.cmd)
//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;










void Device_initGPIO(void)
{
    //
    // This basic pinout includes:
    // PWM1-3, ECAP1, ECAP2, TZ1-TZ4, SPI-A, EQEP1, SCI-A, I2C
    // and a number of I/O pins
    //

    //
    // These can be combined into single statements for improved
    // code efficiency.
    //

    //
    // Enable PWM1-3 on GPIO0-GPIO5
    //
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO4
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO5
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B

    //
    // Enable an GPIO output on GPIO6, set it high
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;   // Enable pullup on GPIO6
    GpioDataRegs.GPASET.bit.GPIO6 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;  // GPIO6 = GPIO6
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;   // GPIO6 = output

    //
    // Enable eCAP2 on GPIO7
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO7
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0; // Synch to SYSCLOUT
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 3;  // GPIO7 = ECAP2

    //
    // Enable GPIO outputs on GPIO8 - GPIO11, set it high
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;   // Enable pullup on GPIO8
    GpioDataRegs.GPASET.bit.GPIO8 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;  // GPIO8 = GPIO8
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO9
    GpioDataRegs.GPASET.bit.GPIO9 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;  // Enable pullup on GPIO10
    GpioDataRegs.GPASET.bit.GPIO10 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GPIO10 = GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;   // GPIO10 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;  // Enable pullup on GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GPIO11 = GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;  // GPIO11 = output

    //
    // Enable Trip Zone inputs on GPIO12 - GPIO15
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPIO12
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO13
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO14
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pullup on GPIO15
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3; // asynch input
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO12 = TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // GPIO13 = TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // GPIO14 = TZ3
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // GPIO15 = ECAP2

    //
    // Enable SPI-A on GPIO16 - GPIO19
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pullup on GPIO16
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pullup on GPIO18
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pullup on GPIO19
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // GPIO16 = SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // GPIO17 = SPIS0MIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // GPIO18 = SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;  // GPIO19 = SPISTEA

    //
    // Enable EQEP1 on GPIO20 - GPIO23
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO20
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO21
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO22
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO23
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Synch to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0; // Synch to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0; // Synch to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0; // Synch to SYSCLKOUT
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;  // GPIO20 = EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;  // GPIO21 = EQEP1B
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;  // GPIO22 = EQEP1S
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;  // GPIO23 = EQEP1I

    //
    // Enable eCAP1 on GPIO24
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO24
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0; // Synch to SYSCLKOUT
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;  // GPIO24 = ECAP1

    //
    // Set input qualifcation period for GPIO25 & GPIO26
    //
    GpioCtrlRegs.GPACTRL.bit.QUALPRD3=1;  // Qual period = SYSCLKOUT/2
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25=2;   // 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26=2;   // 6 samples

    //
    // Make GPIO25 the input source for XINT1
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // GPIO25 = GPIO25
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;   // GPIO25 = input
    GpioIntRegs.GPIOXINT1SEL.all = 25;    // XINT1 connected to GPIO25

    //
    // Make GPIO26 the input source for XINT2
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;  // GPIO26 = GPIO26
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;   // GPIO26 = input
    GpioIntRegs.GPIOXINT2SEL.all = 26;    // XINT2 connected to GPIO26

    //
    // Make GPIO27 wakeup from HALT/STANDBY Low Power Modes
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; // GPIO27 = GPIO27
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;  // GPIO27 = input
    GpioIntRegs.GPIOLPMSEL.bit.GPIO27=1; // GPIO27 will wake the device

    //
    // Qualify GPIO27 by 2 OSCCLK cycles before waking the device from STANDBY
    //
    SysCtrlRegs.LPMCR0.bit.QUALSTDBY=2;

    //
    // Enable SCI-A on GPIO28 - GPIO29
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Enable pullup on GPIO28
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // GPIO28 = SCIRXDA
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Enable pullup on GPIO29
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // GPIO29 = SCITXDA

    //
    // Enable CAN-A on GPIO30 - GPIO31
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;   // Enable pullup on GPIO30
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3; // Asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;  // GPIO30 = CANRXA
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pullup on GPIO31
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;  // GPIO31 = CANTXA

    //
    // Enable I2C-A on GPIO32 - GPIO33
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPIO32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPIO32 = SDAA
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCLA

    //
    // Make GPIO34 an input
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;  // Enable pullup on GPIO34
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // GPIO34 = GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;  // GPIO34 = input
    EDIS;
}






#define TIMEOUT_BYTE_MS         5           /* max timeout between bytes to reset the message in receive communication loop also on timeout triggers evaluation of received till now message */
#define TIMEOUT_MESSAGE_MS      5          /* timeout from the last byte received before say - detected end of the message */
#define TIMEOUT_MESS_MESS_MS    5          /* timeout from the end of previous message to the start of the next message to detect start of message */

#define TIMEOUT_BYTE_TICKS      (((uint32_t)DEVICE_SYSCLK_FREQ * TIMEOUT_BYTE_MS) / 1000)
#define TIMEOUT_MESSAGE_TICKS   (((uint32_t)DEVICE_SYSCLK_FREQ * TIMEOUT_MESSAGE_MS) / 1000)
#define TIMEOUT_MESS_MESS_TICKS (((uint32_t)DEVICE_SYSCLK_FREQ * TIMEOUT_MESS_MESS_MS) / 1000)

uint32_t timerLast;
uint32_t timerNow;
uint32_t timerDiff;
uint32_t timeoutMessageToMessage;
uint32_t timeoutMessage;
uint32_t timeoutByte;
uint16_t timeoutMessageToMessageFlag;
uint16_t timeoutMessageFlag;
uint16_t timeoutByteFlag;

//    /* Check communication timeout */
//    u32TimerNow = timer_get();
//    u32TicksDiff = (uint32_t)(u32TimerLast - u32TimerNow);
//    u64TicksTimeout += u32TicksDiff;
//    u32TimerLast = u32TimerNow;
//    u64TicksTimeoutMatch = (uint64_t)DEVICE_SYSCLK_FREQ;
//    u64TicksTimeoutMatch *=  (uint64_t)COMUNICATION_TIMEOUT_SEC;
//    if (u64TicksTimeout > u64TicksTimeoutMatch)   /* communication timeout in ticks */
//    {
//    }

void modbus_timeout_init(void)
{
    timerLast = timer_get();
    timeoutMessageToMessageFlag = 1;
    timeoutMessageToMessage = 0;
    timeoutMessageFlag = 1;
    timeoutMessage = 0;
    timeoutByteFlag = 1;
    timeoutByte = 0;
}

void modbus_timeout_check()
{
    timerNow = timer_get();

    timerDiff = (uint32_t)(timerLast - timerNow);

    timerLast = timerNow;

    if (timeoutMessageToMessageFlag == 0)
    {
        timeoutMessageToMessage += timerDiff;
        if (timeoutMessageToMessage >= TIMEOUT_MESS_MESS_TICKS)
        {
            timeoutMessageToMessageFlag = 1;
        }
    }
    if (timeoutMessageFlag == 0)
    {
        timeoutMessage += timerDiff;
        if (timeoutMessage >= TIMEOUT_MESSAGE_TICKS)
        {
            timeoutMessageFlag = 1;
        }
    }
    if (timeoutByteFlag == 0)
    {
        timeoutByte += timerDiff;
        if (timeoutByte >= TIMEOUT_BYTE_TICKS)
        {
            timeoutByteFlag = 1;
        }
    }
}

uint16_t modbus_timeout_message_to_message_get()
{
    return timeoutMessageToMessageFlag;
}

void modbus_timeout_message_to_message_reset()
{
    timeoutMessageToMessageFlag = 0;
    timeoutMessageToMessage = 0;
}

uint16_t modbus_timeout_message_get()
{
    return timeoutMessageFlag;
}

void modbus_timeout_message_reset()
{
    timeoutMessageFlag = 0;
    timeoutMessage = 0;
}

uint16_t modbus_timeout_byte_get()
{
    return timeoutByteFlag;
}

void modbus_timeout_byte_reset()
{
    timeoutByteFlag = 0;
    timeoutByte = 0;
}



//each address points to 32-bit register
#define HOLDING_REGISTER_TABLE_COUNT    256
uint32_t holding_32bit_register_address[HOLDING_REGISTER_TABLE_COUNT];





uint32_t holding_register_address_get(uint16_t register_number)
{
    uint16_t index = register_number >> 1;  //support only 32-bit addresses
    if (index < HOLDING_REGISTER_TABLE_COUNT)
    {
        return holding_32bit_register_address[index];
    }
    else
    {
        return 0;
    }

}

void holding_register_address_set(uint16_t register_number, uint32_t data_address)
{
    uint16_t index = register_number >> 1;  //support only 32-bit addresses
    holding_32bit_register_address[index] = data_address;
}




#define DEBUG_RX_BUFFER_LENGTH  32
uint16_t byte_was_processed;
uint16_t debug_process_rx_buffer[DEBUG_RX_BUFFER_LENGTH];
uint16_t debug_process_rx_buffer_count;



uint16_t modbus_uart_error_stat;
uint16_t modbus_timeout_message_to_message_on_no_data_stat;
uint16_t modbus_no_timeout_message_to_message_on_no_data_stat;
uint16_t modbus_process_add_data_stat;
uint16_t modbus_timeout_message_on_rx_data_stat;
uint16_t modbus_timeout_byte_on_rx_data_stat;
uint16_t modbus_no_timeout_on_rx_data_stat;
uint16_t modbus_timeout_message_on_skip_stat;


uint16_t modbus_message_bytes;
uint16_t modbus_message_buffer_rx[256];
uint16_t modbus_message_buffer_tx[256];

uint16_t modbus_message_bytes_skipped;

uint16_t modbus_rx_request[256];
uint16_t modbus_rx_request_length;


uint16_t modbus_rx_bad_crc_count;
uint16_t modbus_rx_bad_id_count;


uint16_t modbus_tx_response_count;
uint16_t modbus_tx_response_index;
uint16_t modbus_tx_response_length;
uint16_t modbus_tx_response[256];



void modbus_init(void)
{
    uint16_t index;

    modbus_uart_error_stat = 0;
    modbus_timeout_message_to_message_on_no_data_stat = 0;
    modbus_no_timeout_message_to_message_on_no_data_stat = 0;
    modbus_process_add_data_stat = 0;
    modbus_timeout_message_on_rx_data_stat = 0;
    modbus_timeout_byte_on_rx_data_stat = 0;
    modbus_no_timeout_on_rx_data_stat = 0;
    modbus_timeout_message_on_skip_stat = 0;


    modbus_message_bytes = 0;
    modbus_message_bytes_skipped = 0;
    modbus_rx_bad_crc_count = 0;
    modbus_rx_bad_id_count = 0;
    modbus_tx_response_count = 0;

    for (index = 0; index < 256; index++)
    {
        modbus_message_buffer_rx[index] = 0;
        modbus_message_buffer_tx[index] = 0;
        modbus_rx_request[index] = 0;
        modbus_tx_response[index] = 0;
    }

    for (index = 0; index < DEBUG_RX_BUFFER_LENGTH; index++)
    {
        debug_process_rx_buffer[index] = 0;
    }
    debug_process_rx_buffer_count = 0;
    byte_was_processed = 1;


}

void modbus_reset_uart_error(void)
{
    modbus_message_bytes = 0;
    modbus_timeout_message_to_message_reset();
    modbus_uart_error_stat++;
}





/* Table of CRC values for high?order byte */
const uint16_t auchCRCHi[256] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low?order byte */
const uint16_t auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

uint16_t modbus_calc_CRC(uint16_t nRxCheckSum, uint16_t nRxChar)
{
    uint16_t uchCRCHi = (uint16_t) (nRxCheckSum >> 8); /* high byte of CRC */
    uint16_t uchCRCLo = (uint16_t) ((nRxCheckSum & 0x00FF) >> 0); /* low byte of CRC */

    uint16_t uIndex = uchCRCLo ^ (nRxChar & 0x00FF);
    uIndex &= 0x00FF;
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
    return ((uint16_t) uchCRCHi << 8 | uchCRCLo);
}



uint16_t modbus_id = 1;

uint16_t modbus_rx_id;
uint16_t modbus_rx_func;
uint16_t modbus_rx_crc;

uint16_t modbus_rx_start_address;
uint16_t modbus_rx_count_registers;


uint16_t modbus_tx_crc;

#define MODBUS_FUNC_READ_HOLDING_REGISTERS_03    0x03
#define MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS_16  0x10

typedef enum
{
    MODBUS_ERR_CMD_OR_MASK                = 0x80,
    MODBUS_ERR_NO_EXCEPTION               = 0x00,
    MODBUS_ERR_ILLEGAL_FUNCTION           = 0x01,
    MODBUS_ERR_ILLEGAL_ADDRESS            = 0x02,
    MODBUS_ERR_ILLEGAL_LENGTH             = 0x03,
    //MODBUS_ERR_SERVER_DEVICE_FAILURE      = 0x04,
    //MODBUS_ERR_RESPONSE_NOT_FIT_IN_BUFFER = 0x10, /* Not ModBus Error Code generally not used and not needed because message length is checked with exception 3 except if used smaller than 256 bytes buffer */
    //MODBUS_ERR_RECEIVE_BUFFER_OVERWRITTEN = 0x11, /* Not ModBus Error Code generally not used but when used shared buffer for Rx and Tx and Request Data Overwritten before read. Possible on File Read Command if multiple records read with bigger length */
    //MODBUS_ERR_DATA_NOT_READY             = 0x12, /* Not ModBus Error Code generally not used only possible on File Read Command when the size of the read record is 0 (no valid data available for this record). Raise Exception need to be configured */
    //MODBUS_ERR_RECEIVE_BYTE_TIMEOUT       = 0x13, /* Not ModBus Error Code generally not used */
    //MODBUS_ERR_RECEIVE_BUFFER_OVERFLOW    = 0x14, /* Not ModBus Error Code generally not used */
    //MODBUS_ERR_RECEIVE_BAD_CS             = 0x15, /* Not ModBus Error Code generally not used */
    //MODBUS_ERR_REQUEST_BAD_LENGTH         = 0x20, /* Impossible Request message length */
}MODBUS_eErrorCodes;

void modbus_evaluate_message_process(void)
{
    uint16_t modbus_crc;
    uint16_t i;
    uint16_t valid_message;
    MODBUS_eErrorCodes exception_code;

    if (modbus_rx_request_length >= 4)
    {
        modbus_rx_id = modbus_rx_request[0];
        modbus_rx_func = modbus_rx_request[1];

        modbus_crc = 0xFFFF;
        for (i = 0; i < modbus_rx_request_length; i++)
        {
            modbus_crc = modbus_calc_CRC(modbus_crc, modbus_rx_request[i]);
        }
        modbus_rx_crc = modbus_crc;

        valid_message = 1;
        if (modbus_rx_crc != 0)
        {
            valid_message = 0;
            modbus_rx_bad_crc_count++;
        }
        if (modbus_rx_id != modbus_id)
        {
            valid_message = 0;
            modbus_rx_bad_id_count++;
        }

        if (valid_message)
        {
            modbus_tx_response_length = 0;
            modbus_tx_response[0] = modbus_id;
            modbus_tx_response[1] = modbus_rx_func;
            exception_code = MODBUS_ERR_NO_EXCEPTION;
            if (modbus_rx_func == MODBUS_FUNC_READ_HOLDING_REGISTERS_03)
            {
                if (modbus_rx_request_length == (2 + 4 + 2))
                {
                    modbus_rx_start_address = ( (uint16_t)modbus_rx_request[2] << 8 ) | (0xFF & modbus_rx_request[3]);
                    modbus_rx_count_registers = ( (uint16_t)modbus_rx_request[4] << 8 ) | (0xFF & modbus_rx_request[5]);

                    //!!!to do size and even register count check
                    if ((modbus_rx_count_registers >= 1) && (modbus_rx_count_registers <= 0x7D) && ((modbus_rx_count_registers & 1) == 0)) //1 to 125 (0x7D)
                    {
                        uint16_t modbus_rx_count_registers_32bit = modbus_rx_count_registers >> 1;
                        uint32_t* pData;
                        uint16_t first_address = 1;
                        modbus_tx_response_length = 3;
                        modbus_tx_response[2] = 0;

                        while (modbus_rx_count_registers_32bit)
                        {
                            pData = (uint32_t*)holding_register_address_get(modbus_rx_start_address);

                            if (pData)
                            {
                                modbus_tx_response[modbus_tx_response_length++] =(*pData >> 24);
                                modbus_tx_response[modbus_tx_response_length++] =(*pData >> 16);
                                modbus_tx_response[modbus_tx_response_length++] =(*pData >>  8);
                                modbus_tx_response[modbus_tx_response_length++] =(*pData >>  0);
                                modbus_tx_response[2] += 4;

                                modbus_rx_start_address += 2;
                                modbus_rx_count_registers_32bit--;
                                first_address = 0;
                            }
                            else
                            {
                                if (first_address)
                                {
                                    exception_code = MODBUS_ERR_ILLEGAL_ADDRESS;
                                }
                                else
                                {
                                    exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                                }
                                break;
                            }
                        }

                    }
                    else
                    {
                        exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                    }

                }
                else
                {
                    //exception_code = MODBUS_ERR_REQUEST_BAD_LENGTH;
                    exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                }
            }
            else
            if (modbus_rx_func == MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS_16)
            {
                if (modbus_rx_request_length >= (2 + 5 + 2))
                {
                    modbus_rx_start_address = ( (uint16_t)modbus_rx_request[2] << 8 ) | (0xFF & modbus_rx_request[3]);
                    modbus_rx_count_registers = ( (uint16_t)modbus_rx_request[4] << 8 ) | (0xFF & modbus_rx_request[5]);

                    if (modbus_rx_request[6] != modbus_rx_count_registers * 2)
                    {
                        //exception_code = MODBUS_ERR_REQUEST_BAD_LENGTH;
                        exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                    }
                    else
                    if (modbus_rx_request[6] != (modbus_rx_request_length - (2 + 5 + 2)))
                    {
                        //exception_code = MODBUS_ERR_REQUEST_BAD_LENGTH;
                        exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                    }
                    else
                    {
                        if ((modbus_rx_count_registers >= 1) && (modbus_rx_count_registers <= 0x7B) && ((modbus_rx_count_registers & 1) == 0) ) //1 to 123 (0x7B)
                        {
                            uint16_t modbus_rx_count_registers_32bit;
                            uint16_t modbus_rx_start_address_32bit;
                            uint32_t u32Data;
                            uint16_t index_rx;
                            uint32_t* pData;
                            uint16_t first_address = 1;

                            //fill preliminary if success answer
                            modbus_tx_response[2] = modbus_rx_request[2];
                            modbus_tx_response[3] = modbus_rx_request[3];
                            modbus_tx_response[4] = modbus_rx_request[4];
                            modbus_tx_response[5] = modbus_rx_request[5];
                            modbus_tx_response_length = 6;

                            //check all addresses valid in table
                            modbus_rx_start_address_32bit = modbus_rx_start_address;
                            modbus_rx_count_registers_32bit = modbus_rx_count_registers >> 1;
                            while (modbus_rx_count_registers_32bit)
                            {
                                pData = (uint32_t*)holding_register_address_get(modbus_rx_start_address_32bit);

                                if (pData)
                                {
                                    modbus_rx_start_address_32bit += 2;
                                    modbus_rx_count_registers_32bit--;
                                    first_address = 0;
                                }
                                else
                                {
                                    if (first_address)
                                    {
                                        exception_code = MODBUS_ERR_ILLEGAL_ADDRESS;
                                    }
                                    else
                                    {
                                        exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                                    }
                                    break;
                                }
                            }

                            index_rx = 7;

                            if (exception_code == MODBUS_ERR_NO_EXCEPTION)
                            {
                                modbus_rx_start_address_32bit = modbus_rx_start_address;
                                modbus_rx_count_registers_32bit = modbus_rx_count_registers >> 1;
                                while (modbus_rx_count_registers_32bit)
                                {
                                    pData = (uint32_t*)holding_register_address_get(modbus_rx_start_address_32bit);

                                    if (pData)
                                    {
                                        u32Data  = ((uint32_t)modbus_rx_request[index_rx++] << 24);
                                        u32Data += ((uint32_t)modbus_rx_request[index_rx++] << 16);
                                        u32Data += ((uint32_t)modbus_rx_request[index_rx++] <<  8);
                                        u32Data += ((uint32_t)modbus_rx_request[index_rx++] <<  0);

                                        *pData = u32Data;

                                        modbus_rx_start_address_32bit += 2;
                                        modbus_rx_count_registers_32bit--;

//                                        modbus_tx_response[modbus_tx_response_length++] =(*pData >> 24);
//                                        modbus_tx_response[modbus_tx_response_length++] =(*pData >> 16);
//                                        modbus_tx_response[modbus_tx_response_length++] =(*pData >>  8);
//                                        modbus_tx_response[modbus_tx_response_length++] =(*pData >>  0);
//                                        modbus_tx_response[2] += 4;

                                    }
                                }
                            }

                        }
                        else
                        {
                            exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                        }
                    }

                }
                else
                {
                    //exception_code = MODBUS_ERR_REQUEST_BAD_LENGTH;
                    exception_code = MODBUS_ERR_ILLEGAL_LENGTH;
                }

            }
            else
            {
                exception_code = MODBUS_ERR_ILLEGAL_FUNCTION;
            }

            if (exception_code)
            {
                modbus_tx_response[1] |= MODBUS_ERR_CMD_OR_MASK;
                modbus_tx_response[2] = exception_code;
                modbus_tx_response_length = 3;
            }

            if (modbus_tx_response_length)
            {

                modbus_crc = 0xFFFF;
                for (i = 0; i < modbus_tx_response_length; i++)
                {
                    modbus_crc = modbus_calc_CRC(modbus_crc, modbus_tx_response[i]);
                }
                modbus_tx_crc = modbus_crc;

                /* store checksum after the message */
                modbus_tx_response[modbus_tx_response_length] =  (uint16_t)(modbus_tx_crc & 0x00FF);
                modbus_tx_response_length++;
                modbus_tx_response[modbus_tx_response_length] =  (uint16_t )(modbus_tx_crc >> 8);
                modbus_tx_response_length++;

                modbus_tx_response_index = 0;
                if (modbus_tx_response_count == 0)  //previous message finished
                {
                    modbus_tx_response_count = modbus_tx_response_length;
                }



            }
        }

        modbus_rx_request_length = 0;
    }
}


void modbus_copy_to_evaluate_message(void)
{
    uint16_t index;

    if (modbus_message_bytes)
    {
        for (index = 0; index < modbus_message_bytes; index++)
        {
            modbus_rx_request[index] = modbus_message_buffer_rx[index];
        }
        modbus_rx_request_length = modbus_message_bytes;
    }
}

void modbus_skip(void)
{
    modbus_timeout_check();
    if (modbus_message_bytes == 0)
    {

    }
    else
    {
        if (modbus_timeout_message_get())
        {
            modbus_timeout_message_on_skip_stat++;
            modbus_timeout_message_reset();
            modbus_timeout_message_to_message_reset();
            modbus_copy_to_evaluate_message();
            modbus_message_bytes = 0;
        }
    }

}

void modbus_process(char data)
{
    uint16_t add_data = 0;
    if (modbus_message_bytes == 0)
    {
        modbus_timeout_check();
        if (modbus_timeout_message_to_message_get())
        {
            modbus_timeout_message_to_message_on_no_data_stat++;
            modbus_timeout_message_to_message_reset();
            modbus_timeout_message_reset();
            modbus_timeout_byte_reset();
            add_data = 1;
        }
        else
        {
            modbus_no_timeout_message_to_message_on_no_data_stat++;
        }
    }
    else
    {
        modbus_timeout_check();
        if (modbus_timeout_message_get())
        {
            modbus_timeout_message_on_rx_data_stat++;
            modbus_timeout_message_reset();
            modbus_timeout_message_to_message_reset();
            modbus_copy_to_evaluate_message();
            modbus_message_bytes = 0;
        }
        else
        if (modbus_timeout_byte_get())
        {
            modbus_timeout_byte_on_rx_data_stat++;
            modbus_timeout_byte_reset();
            modbus_timeout_message_to_message_reset();
            modbus_copy_to_evaluate_message();
            modbus_message_bytes = 0;
        }
        else
        {
            modbus_no_timeout_on_rx_data_stat++;
            modbus_timeout_message_to_message_reset();
            modbus_timeout_message_reset();
            modbus_timeout_byte_reset();
            add_data = 1;
        }
    }

    if (add_data)
    {
        modbus_process_add_data_stat++;
        if (modbus_message_bytes < sizeof(modbus_message_buffer_rx))
        {
            modbus_message_buffer_rx[modbus_message_bytes] = data;
            modbus_message_bytes++;
        }
        else
        {
            modbus_message_bytes_skipped++;
        }
    }

}

uint16_t smallUartBuffer[32];
uint16_t smallUartBufferIndexWr;
uint16_t smallUartBufferIndexRd;

void process_scia_buffer(void)
{
    uint16_t no_data = 1;
    if (smallUartBufferIndexWr >= sizeof(smallUartBuffer))
    {
        return;
    }
    if (smallUartBufferIndexRd >= sizeof(smallUartBuffer))
    {
        smallUartBufferIndexRd = 0;
    }
    //if (smallUartBufferIndexWr != smallUartBufferIndexRd)
    while (smallUartBufferIndexWr != smallUartBufferIndexRd)
    {
        byte_was_processed = 1;
        if (debug_process_rx_buffer_count < DEBUG_RX_BUFFER_LENGTH)
        {
            debug_process_rx_buffer[debug_process_rx_buffer_count] = 2;
            debug_process_rx_buffer_count++;
        }

        modbus_process(smallUartBuffer[smallUartBufferIndexRd]);
        if (smallUartBufferIndexRd >= (sizeof(smallUartBuffer)-1))
        {
            smallUartBufferIndexRd = 0;
        }
        else
        {
            smallUartBufferIndexRd++;
        }
        no_data = 0;
    }
    //else
    if (no_data)
    {
        if (byte_was_processed)
        {
            byte_was_processed = 0;
            if (debug_process_rx_buffer_count < DEBUG_RX_BUFFER_LENGTH)
            {
                debug_process_rx_buffer[debug_process_rx_buffer_count] = 1;
                debug_process_rx_buffer_count++;
            }
        }

        modbus_skip();
    }
}



void process_scia_tx_data(void)
{

    if (modbus_tx_response_count)
    {
        uint16_t sentData;
        do
        {
            sentData = 0;
            if(scia_is_tx_possible())
            {
                if (modbus_tx_response_index < modbus_tx_response_count)
                {
                    sentData = 1;
                    scia_tx_data(modbus_tx_response[modbus_tx_response_index]);
                    modbus_tx_response_index++;
                }
                if (modbus_tx_response_index == modbus_tx_response_count)
                {
                    modbus_tx_response_count = 0;
                }
            }
        } while(sentData);

    }
    else
    {
        scia_CheckRTSDisable();
    }


}


void process_scia_init(void)
{
    smallUartBufferIndexWr = 0;
    smallUartBufferIndexRd = 0;
}


void process_scia_rx(void)
{
    char cRecvData;
    uint16_t u16ReadResult;

    do
    {
        u16ReadResult = scia_ReadChar(&cRecvData);

        if (u16ReadResult == 1)
        {
            if (smallUartBufferIndexWr >= sizeof(smallUartBuffer))
            {
                smallUartBufferIndexWr = 0;
            }
            smallUartBuffer[smallUartBufferIndexWr] = cRecvData;
            if (smallUartBufferIndexWr >= (sizeof(smallUartBuffer)-1))
            {
                smallUartBufferIndexWr = 0;
            }
            else
            {
                smallUartBufferIndexWr++;
            }
        }
        else
        if (u16ReadResult == 2)
        {
            modbus_reset_uart_error();
        }

    } while (u16ReadResult == 1);

}


uint32_t holding_registers_data[64];

void holding_register_address_init(void)
{
    uint16_t index;
    uint32_t initial_data = 0x12345678;
    for(index = 0; index < 64; index++)
    {
        holding_registers_data[index] = initial_data++;
        holding_register_address_set(index << 1, (uint32_t)&holding_registers_data[index]);
    }

}





uint32_t delay;

//
// Main
//
void main(void)
{

//    DisableDog();
//    inreset = 1;
//    while(inreset)
//    {
//    }

    #if USE_SENDSTRING
    uint16_t u16WaitSeconds = 0;
    //uint32_t u32TicksTimerAlive = 0;
    char pSeconds[3+1];
    pSeconds[0] = '\r';
    pSeconds[3] = 0;
    #endif



    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
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
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    //PieVectTable.EPWM1_INT = &epwm1_timer_isr;
    //PieVectTable.EPWM2_INT = &epwm2_timer_isr;
    //PieVectTable.EPWM3_INT = &epwm3_timer_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    Device_initGPIO();

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    // InitPeripherals();  // Not required for this example
    //InitEPwmTimer();    // For this example, only initialize the ePWM Timers

    //
    // Step 5. User specific code, enable interrupts
    //

    //
    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: epwm1_timer_isr(), 
    // epwm2_timer_isr(), epwm3_timer_isr and and InitFlash();
    // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the F2808.cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

    //
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //
    InitFlash();

//    //
//    // Initalize counters
//    //
//    //EPwm1TimerIntCount = 0;
//    //EPwm2TimerIntCount = 0;
//    //EPwm3TimerIntCount = 0;
//    LoopCount = 0;
//
//
//


    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT
    //
    //IER |= M_INT3;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    //
    //PieCtrlRegs.PIEIER3.bit.INTx1 = PWM1_INT_ENABLE;
    //PieCtrlRegs.PIEIER3.bit.INTx2 = PWM2_INT_ENABLE;
    //PieCtrlRegs.PIEIER3.bit.INTx3 = PWM3_INT_ENABLE;


//    #if USE_LEDS
//    // Enable a GPIO output on DEVICE_GPIO_PIN_LED1, set it high
//    led_pin_write(DEVICE_GPIO_PIN_LED1, 1);
//    led_setup(DEVICE_GPIO_PIN_LED1);
//
//    // Enable a GPIO output on DEVICE_GPIO_PIN_LED2, set it high
//    led_pin_write(DEVICE_GPIO_PIN_LED2, 1);
//    led_setup(DEVICE_GPIO_PIN_LED2);
//    #endif
//
    #if USE_TXEN
    led_pin_write(DEVICE_GPIO_PIN_SCITXEN, 0);
    led_setup(DEVICE_GPIO_PIN_SCITXEN);
    #endif

    /* Initialize modbus interface */
    scia_init();
    timer_init();
    modbus_init();
    modbus_timeout_init();
    process_scia_init();

    holding_register_address_init();



    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    //
    // Step 6. IDLE loop. Just sit and loop forever (optional)
    //
//    EALLOW;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
//    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
//    EDIS;


    for(;;)
    {

        process_scia_rx();
        process_scia_buffer();
        modbus_evaluate_message_process();
        process_scia_tx_data();

//        //
//        // This loop will be interrupted, so the overall delay between pin
//        // toggles will be longer.
//        //
//        LoopCount++;
//        //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//        #if USE_LEDS
//        led_pin_write (DEVICE_GPIO_PIN_LED1,0);
//        #endif
//        DELAY_US(DELAY);
//        #if USE_LEDS
//        led_pin_write (DEVICE_GPIO_PIN_LED2,0);
//        #endif
//        DELAY_US(DELAY);
//        #if USE_LEDS
//        led_pin_write (DEVICE_GPIO_PIN_LED1,1);
//        #endif
//        DELAY_US(DELAY);
//        #if USE_LEDS
//        led_pin_write (DEVICE_GPIO_PIN_LED2,1);
//        #endif

        #if USE_SENDSTRING
        DELAY_US(DELAY);
        u16WaitSeconds++;
        #if USE_LEDS
        //led_pin_toggle(DEVICE_GPIO_PIN_LED2);
        #endif
        pSeconds[1] = ((u16WaitSeconds / 10) > 0) ? (u16WaitSeconds / 10) + 0x30 : ' ';
        pSeconds[2] = (u16WaitSeconds % 10) + 0x30;
        scia_SendString(pSeconds);
        #endif


//        delay = DataArray[LoopCount];
//        delay &= 0xFF;
//        DELAY_US(delay);
    }
}

////
//// InitEPwmTimer -
////
//void
//InitEPwmTimer()
//{
//    EALLOW;
//    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
//    EDIS;
//
//    InitEPwm1Gpio();
//    InitEPwm2Gpio();
//    InitEPwm3Gpio();
//
//    //
//    // Setup Sync
//    //
//    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through
//    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through
//    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through
//
//    //
//    // Allow each timer to be sync'ed
//    //
//    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;
//    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
//    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
//
//    EPwm1Regs.TBPHS.half.TBPHS = 100;
//    EPwm2Regs.TBPHS.half.TBPHS = 200;
//    EPwm3Regs.TBPHS.half.TBPHS = 300;
//
//    EPwm1Regs.TBPRD = PWM1_TIMER_TBPRD;
//    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;    // Count up
//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//    EPwm1Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
//    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
//
//    EPwm2Regs.TBPRD = PWM2_TIMER_TBPRD;
//    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
//    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Enable INT on Zero event
//    EPwm2Regs.ETSEL.bit.INTEN = PWM2_INT_ENABLE;   // Enable INT
//    EPwm2Regs.ETPS.bit.INTPRD = ET_2ND;            // Generate INT on 2nd event
//
//    EPwm3Regs.TBPRD = PWM3_TIMER_TBPRD;
//    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
//    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Enable INT on Zero event
//    EPwm3Regs.ETSEL.bit.INTEN = PWM3_INT_ENABLE;   // Enable INT
//    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event
//
//    EALLOW;
//    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
//    EDIS;
//}
//
////
//// epwm1_timer_isr - This ISR MUST be executed from RAM as it will put the
//// Flash into Sleep Interrupt routines uses in this example:
////
//interrupt void
//epwm1_timer_isr(void)
//{
//    //
//    // Put the Flash to sleep
//    //
//    EALLOW;
//    FlashRegs.FPWR.bit.PWR = FLASH_SLEEP;
//    EDIS;
//
//    EPwm1TimerIntCount++;
//
//    //
//    // Clear INT flag for this timer
//    //
//    EPwm1Regs.ETCLR.bit.INT = 1;
//
//    //
//    // Acknowledge this interrupt to receive more interrupts from group 3
//    //
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//}
//
////
//// epwm2_timer_isr - This ISR MUST be executed from RAM as it will put the
//// Flash into Standby
////
//interrupt void
//epwm2_timer_isr(void)
//{
//    EPwm2TimerIntCount++;
//
//    //
//    // Put the Flash into standby
//    //
//    EALLOW;
//    FlashRegs.FPWR.bit.PWR = FLASH_STANDBY;
//    EDIS;
//
//    //
//    // Clear INT flag for this timer
//    //
//    EPwm2Regs.ETCLR.bit.INT = 1;
//
//    //
//    // Acknowledge this interrupt to receive more interrupts from group 3
//    //
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//}
//
////
//// epwm3_timer_isr -
////
//interrupt void
//epwm3_timer_isr(void)
//{
//    Uint16 i;
//
//    EPwm3TimerIntCount++;
//
//    //
//    // Short Delay to simulate some ISR Code
//    //
//    for(i = 1; i < 0x01FF; i++)
//    {
//
//    }
//
//    //
//    // Clear INT flag for this timer
//    //
//    EPwm3Regs.ETCLR.bit.INT = 1;
//
//    //
//    // Acknowledge this interrupt to receive more interrupts from group 3
//    //
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//}

//
// End of File
//

