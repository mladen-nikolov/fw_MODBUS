/* *****************************************************************************
 * File:   gpio.h
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
#include "config.h"
#include <stdint.h>

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

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */
void gpio_pin_toggle(uint32_t pin);
void gpio_pin_write(uint32_t pin, uint32_t level);
void gpio_setup(uint32_t PinNumber);


#ifdef __cplusplus
}
#endif /* __cplusplus */


