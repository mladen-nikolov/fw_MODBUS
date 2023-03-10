/* *****************************************************************************
 * File:   scia.c
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
#include "scia.h"
#include "device.h"
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

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */


void scia_fifo_init(void)
{
    SciaRegs.SCIFFTX.all=0x0000;
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}

void scia_echoback_init(void)
{
    uint32_t baud_rate = CONFIG_BAUD;
    uint32_t lpclk = DEVICE_SYSCLK_FREQ;

    uint32_t divider;

    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

    // 1 stop bit,  No loopback, even parity,8 char bits, async mode,
    SciaRegs.SCICCR.all =0x0007;
    SciaRegs.SCICCR.bit.PARITY = 1;
    SciaRegs.SCICCR.bit.PARITYENA = 1;

    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL1.all =0x0003;

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;

    // lpclk(@LSPCLK) = 90 MHz (1/1 of 90 MHz SYSCLK)
    divider =  ((uint32_t)lpclk + ((baud_rate * 8)/2)) / ((uint32_t)baud_rate * 8) - 1;

    SciaRegs.SCIHBAUD    =(divider >> 8) & 0xFF;
    SciaRegs.SCILBAUD    =((uint16_t)divider & 0xFF);

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1; //ofv clear
    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scia_init(void)
{
    #if CONFIG_USE_TXEN
    gpio_pin_write(CONFIG_SCITXEN_PIN, 0);
    gpio_setup(CONFIG_SCITXEN_PIN);
    #endif

    EALLOW;
    // Enable SCI-A on GPIO28 - GPIO29
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Enable pullup on GPIO28
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // GPIO28 = SCIRXDA
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Enable pullup on GPIO29
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // GPIO29 = SCITXDA
    EDIS;

    scia_fifo_init();
    scia_echoback_init();
    return;
}

void scia_flush(void)
{
    // wait while TX is busy.
    while(SciaRegs.SCIFFTX.bit.TXFFST > 0)
    {
    }
}

#if CONFIG_USE_DEBUG_SEND
void scia_SendString(char* pData)
{
    uint16_t* pu16Data = (uint16_t*)pData;
    while(pu16Data[0] & 0xFF)
    {
        scia_tx_data(*pu16Data++);
    }
    scia_flush();
}
#endif

//
// End of File
//





