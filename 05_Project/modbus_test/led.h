/*
 * led.h
 *
 *  Created on: Jul 11, 2022
 *      Author: Dimitar Lilov
 */

#ifndef DEVICE_F2838X_LED_H_
#define DEVICE_F2838X_LED_H_

#include "config.h"
#include <stdint.h>

void led_pin_toggle(uint32_t pin);
void led_pin_write(uint32_t pin, uint32_t level);
void led_setup(uint32_t PinNumber);

#endif /* DEVICE_F2838X_LED_H_ */
