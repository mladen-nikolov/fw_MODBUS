/*
 * gpio.h
 *
 *  Created on: Jul 11, 2022
 *      Author: Dimitar Lilov
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "config.h"
#include <stdint.h>

void gpio_pin_toggle(uint32_t pin);
void gpio_pin_write(uint32_t pin, uint32_t level);
void gpio_setup(uint32_t PinNumber);

#endif /* GPIO_H_ */
