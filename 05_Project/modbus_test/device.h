/*
 * device.h
 *
 *  Created on: Jul 28, 2022
 *      Author: Dimitar Lilov
 */

#ifndef DEVICE_H_
#define DEVICE_H_


#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
//#include "Flash2806x_API_Library.h"

//#include "driverlib.h"



//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define SECTION_RAM_FUNC    "ramfuncs"


#define USE_LEDS                        0

#define USE_TXEN                        1

#define DEVICE_GPIO_PIN_SCITXEN         15U

//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************
#define DEVICE_SYSCLK_FREQ  90000000    /* 90 MHz */



#endif /* DEVICE_H_ */
