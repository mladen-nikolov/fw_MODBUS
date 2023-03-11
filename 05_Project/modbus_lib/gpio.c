/* *****************************************************************************
 * File:   gpio.c
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
#include "device.h"
#include "gpio.h"

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

void gpio_pin_toggle(uint32_t pin)
{
    if (pin < 32)
    {
        GpioDataRegs.GPATOGGLE.all |= (1 << pin);
    }
    else
    {
        GpioDataRegs.GPBTOGGLE.all |= (1 << (pin-32));
    }
}

void gpio_pin_write(uint32_t pin, uint32_t level)
{
    if (level == 1)
    {
        if (pin < 32)
        {
            GpioDataRegs.GPASET.all |= (1 << pin);
        }
        else
        {
            GpioDataRegs.GPBSET.all |= (1 << (pin-32));
        }

    }
    else
    {
        if (pin < 32)
        {
            GpioDataRegs.GPACLEAR.all |= (1 << pin);
        }
        else
        {
            GpioDataRegs.GPBCLEAR.all |= (1 << (pin-32));
        }
    }
}

void gpio_setup(uint32_t PinNumber)
{
    Uint32 mask;

    if(PinNumber > (Uint16)47)
    {
        asm("    ESTOP0");  // Stop here. Invalid option.
        for(;;);
    }

    // Pins GPIO32-GPIO47
    else if(PinNumber >= 32)
    {
        EALLOW;
        mask = ~( ((Uint32)1 << (PinNumber-32)*2) | ((Uint32)1 << (PinNumber-32)*2+1) );
        GpioCtrlRegs.GPBMUX1.all &= mask;
        GpioCtrlRegs.GPBDIR.all = GpioCtrlRegs.GPBDIR.all | ((Uint32)1 << (PinNumber-32) );
        EDIS;
    }

    // Pins GPIO16-GPIO31
    else if(PinNumber >= 16)
    {
        EALLOW;
        mask = ~( ((Uint32)1 << (PinNumber-16)*2) | ((Uint32)1 << (PinNumber-16)*2+1) );
        GpioCtrlRegs.GPAMUX2.all &= mask;
        GpioCtrlRegs.GPADIR.all = GpioCtrlRegs.GPADIR.all | ((Uint32)1 << PinNumber);
        EDIS;
    }

    // Pins GPIO0-GPIO15
    else
    {
        EALLOW;
        mask = ~( ((Uint32)1 << PinNumber*2) | ((Uint32)1 << PinNumber*2+1 ));
        GpioCtrlRegs.GPAMUX1.all &= mask;
        GpioCtrlRegs.GPADIR.all = GpioCtrlRegs.GPADIR.all | ((Uint32)1 << PinNumber);
        EDIS;
    }
}


