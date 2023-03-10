/* *****************************************************************************
 * File:   scia.h
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
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "device.h"
#if CONFIG_USE_TXEN
#include "gpio.h"
#endif

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
 * Function-Like Macro
 **************************************************************************** */
#define scia_is_rx_available()  (SciaRegs.SCIFFRX.bit.RXFFST > 0)
#define scia_rx_data()          (SciaRegs.SCIRXBUF.all)
#define scia_is_tx_possible()   (SciaRegs.SCIFFTX.bit.TXFFST < 4)
#define scia_is_tx_complete()   (SciaRegs.SCICTL2.bit.TXEMPTY == 1)
#define scia_rx_is_ovf()        (SciaRegs.SCIFFRX.bit.RXFFOVF == 1)
#define scia_rx_ovf_clear()     {SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1; SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;}

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
void scia_init(void);
void scia_flush(void);
void scia_ovf_check(void);

/* *****************************************************************************
 * Functions
 **************************************************************************** */
#if CONFIG_USE_DEBUG_SEND
void scia_SendString(char* pData);
#else
#define scia_SendString(x)
#endif

#if CONFIG_USE_TXEN
static inline void scia_CheckRTSDisable(void)
{
    if(scia_is_tx_complete())
    {
        gpio_pin_write(CONFIG_SCITXEN_PIN, 0);
    }
}
#endif

static inline void scia_tx_data(char data)
{
    #if CONFIG_USE_TXEN
    gpio_pin_write(CONFIG_SCITXEN_PIN, 1);
    #endif
    SciaRegs.SCITXBUF = (data & 0xFF);
}

static inline void scia_SendChar(char *pData)
{
    uint16_t* pu16Data = (uint16_t*)pData;
    while (scia_is_tx_possible() == false)
    {

    }
    #if CONFIG_USE_TXEN
    gpio_pin_write(CONFIG_SCITXEN_PIN, 1);
    #endif
    SciaRegs.SCITXBUF=(*pu16Data & 0xFF);
}


static inline uint16_t scia_ReadChar(char* pData)
{
    uint16_t u16Return = 0;

    if (scia_rx_is_ovf())
    {
        scia_rx_ovf_clear();

        SciaRegs.SCIFFTX.all=0x0000;
        SciaRegs.SCIFFTX.all=0xE040;
        SciaRegs.SCIFFRX.all=0x6044;
        SciaRegs.SCIFFCT.all=0x0;

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



#ifdef __cplusplus
}
#endif /* __cplusplus */









