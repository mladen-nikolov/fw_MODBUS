/*
 * led.c
 *
 *  Created on: Jul 11, 2022
 *      Author: Dimitar Lilov
 */


#include "device.h"
#include "led.h"




void led_pin_toggle(uint32_t pin)
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


void led_pin_write(uint32_t pin, uint32_t level)
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



void led_setup(uint32_t PinNumber)
{
    Uint32 mask;

    // Before calling the Toggle Test, we must setup
    // the MUX and DIR registers.

    //led_pin_write(PinNumber, 1);

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


