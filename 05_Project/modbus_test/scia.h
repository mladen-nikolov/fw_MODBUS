/*
 * scia.h
 *
 *  Created on: Jun 27, 2022
 *      Author: Dimitar Lilov
 */

#ifndef SCIA_H_
#define SCIA_H_

//
// Includes
//
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "device.h"
#if USE_TXEN
#include "gpio.h"
#endif

//
// Configuration Defines
//

//
// Defines
//
#define ACK                             0x2D
#define NAK                             0xA5
#define STX                             0x02
#define ETX                             0x03
#define CR                              0x0D
#define LF                              0x0A

extern uint32_t scia_fail_ovf;

//
// Function Prototypes
//
void scia_init(void);
void scia_flush(void);
void scia_ovf_check(void);

#define scia_is_rx_available()  (SciaRegs.SCIFFRX.bit.RXFFST > 0)
#define scia_rx_data()          (SciaRegs.SCIRXBUF.all)
#define scia_is_tx_possible()   (SciaRegs.SCIFFTX.bit.TXFFST < 4)
#define scia_is_tx_complete()   (SciaRegs.SCICTL2.bit.TXEMPTY == 1)
#define scia_rx_is_ovf()        (SciaRegs.SCIFFRX.bit.RXFFOVF == 1)
#define scia_rx_ovf_clear()     {SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1; SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;}

#if USE_SENDSTRING
void scia_SendString(char* pData);
#else
#define scia_SendString(x)
#endif

#if USE_TXEN
static inline
void scia_CheckRTSDisable(void)
{
    if(scia_is_tx_complete())
    {
        gpio_pin_write(DEVICE_GPIO_PIN_SCITXEN, 0);
    }
}
#endif

static inline
void scia_tx_data(char data)
{
    #if USE_TXEN
    gpio_pin_write(DEVICE_GPIO_PIN_SCITXEN, 1);
    #endif
    SciaRegs.SCITXBUF = (data & 0xFF);
}

static inline
void scia_SendChar(char *pData)
{
    uint16_t* pu16Data = (uint16_t*)pData;
    while (scia_is_tx_possible() == false)
    {

    }
    #if USE_TXEN
    gpio_pin_write(DEVICE_GPIO_PIN_SCITXEN, 1);
    #endif
    SciaRegs.SCITXBUF=(*pu16Data & 0xFF);
}

void
scia_fifo_init(void);

static inline uint16_t scia_ReadChar(char* pData)
{
    uint16_t u16Return = 0;

    if (scia_rx_is_ovf())
    {
        //fail_scia_ovf++;
        //while(scia_is_rx_available())
        //{
        //    *pData = (scia_rx_data() & 0xFF);
        //}
        scia_rx_ovf_clear();


        //scia_fifo_init();
        SciaRegs.SCIFFTX.all=0x0000;
        SciaRegs.SCIFFTX.all=0xE040;
        SciaRegs.SCIFFRX.all=0x6044;
        //SciaRegs.SCIFFRX.all=0x4044; //with overflow clear
        SciaRegs.SCIFFCT.all=0x0;


        //while(scia_is_rx_available())
        //{
        //    *pData = (scia_rx_data() & 0xFF);
        //}
        u16Return = 2;
    }
    else
    if(scia_is_rx_available())
    {
        *pData = (scia_rx_data() & 0xFF);
        u16Return = 1;
    }
    return u16Return;
}



#endif /* SCIA_H_ */
//
// End of File
//




