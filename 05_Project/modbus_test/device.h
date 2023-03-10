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


#define USE_LEDS                    0

////CS1291
////#define DEVICE_GPIO_PIN_LED1        17U   // SPI_SOMI
////#define DEVICE_GPIO_PIN_LED2        16U   // SPI-SIMO
//#define DEVICE_GPIO_PIN_LED1        23U   // EQEP1I
//#define DEVICE_GPIO_PIN_LED2        22U   // EQEP1S

//LaunchXL
//#define DEVICE_GPIO_PIN_LED1        39U   // GPIO number for LD1 (D10 LaunchXL)
//#define DEVICE_GPIO_PIN_LED2        34U   // GPIO number for LD2 (D9  LaunchXL)
//#define DEVICE_GPIO_PIN_LED3        145U  // GPIO number for LD7 (EtherCAT Error LED)
//#define DEVICE_GPIO_PIN_LED4        146U  // GPIO number for LD8 (EtherCAT Run LED)

#define USE_TXEN                    1

#define DEVICE_GPIO_PIN_SCITXEN        15U

//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************
#define DEVICE_SYSCLK_FREQ  90000000    /* 90 MHz */



#endif /* DEVICE_H_ */
