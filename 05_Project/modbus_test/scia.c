/*
 * scia.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Dimitar Lilov
 */


//
// Included Files
//
#include "scia.h"
#include "device.h"
#include "config.h"


uint32_t scia_fail_ovf;

//
// scia_fifo_init - Initalize the SCI FIFO
//
void
 scia_fifo_init()
{
    scia_fail_ovf = 0;

//    Uint16  TXFFIL:5;           // 4:0  Interrupt level
//    Uint16  TXFFIENA:1;         // 5    Interrupt enable
//    Uint16  TXFFINTCLR:1;       // 6    Clear INT flag
//    Uint16  TXFFINT:1;          // 7    INT flag
//    Uint16  TXFFST:5;           // 12:8 FIFO status
//    Uint16  TXFIFOXRESET:1;     // 13   FIFO reset
//    Uint16  SCIFFENA:1;         // 14   Enhancement enable
//    Uint16  SCIRST:1;           // 15   SCI reset rx/tx channels
//
    SciaRegs.SCIFFTX.all=0x0000;
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    //SciaRegs.SCIFFRX.all=0x4044; //with overflow clear
    SciaRegs.SCIFFCT.all=0x0;
}


uint32_t baud_rate = CFG_BAUD;
uint32_t lpclk = DEVICE_SYSCLK_FREQ;

//
// scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x0103,
// default, 1 STOP bit, no parity
//
void
scia_echoback_init()
{
    uint32_t divider;
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    //
    // 1 stop bit,  No loopback, even parity,8 char bits, async mode,
    // idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;
    SciaRegs.SCICCR.bit.PARITY = 1;
    SciaRegs.SCICCR.bit.PARITYENA = 1;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;


    baud_rate = CFG_BAUD;
    lpclk = DEVICE_SYSCLK_FREQ;

    //
    // 1000000 baud @LSPCLK = 22.5 MHz (1/4 of 90 MHz SYSCLK)
    //uint32_t divider =  ((uint32_t)22500000 + ((1000000 * 8)/2)) / ((uint32_t)1000000 * 8) - 1;
    //
    // 1000000 baud @LSPCLK = 90 MHz (1/1 of 90 MHz SYSCLK)
    //uint32_t divider =  ((uint32_t)90000000 + ((1000000 * 8)/2)) / ((uint32_t)1000000 * 8) - 1;

    //
    // lpclk(@LSPCLK) = 90 MHz (1/1 of 90 MHz SYSCLK)
    divider =  ((uint32_t)lpclk + ((baud_rate * 8)/2)) / ((uint32_t)baud_rate * 8) - 1;

    SciaRegs.SCIHBAUD    =(divider >> 8) & 0xFF;
    SciaRegs.SCILBAUD    =((uint16_t)divider & 0xFF);

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1; //ofv clear



//    SciaRegs.SCIFFTX.bit.TXFFINTCLR =1;
//    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;       // Transmit FIFO reset;
//
//    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;        // Receive FIFO reset;
//    SciaRegs.SCIFFRX.bit.RXFFINTCLR =1;

//    ScibRegs.SCIFFTX.bit.TXFFINTCLR =1;
//    ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;       // Transmit FIFO reset;
//
//    ScibRegs.SCIFFRX.bit.RXFIFORESET=1;        // Receive FIFO reset;
//    ScibRegs.SCIFFRX.bit.RXFFINTCLR =1;




    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

}


//
// scia_init - Initialize the SCI-A port for communications with the host.
//
void scia_init(void)
{
    scia_fifo_init();
    scia_echoback_init();

    return;
}



//
// scia_flush - This routine flushes SCIA.
//
void scia_flush(void)
{

    //
    // wait while TX is busy.
    //
    while(SciaRegs.SCIFFTX.bit.TXFFST > 0)
    {
    }

}


//
// scia_SendString - Sends the Global checksum value
//
#if USE_SENDSTRING
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





