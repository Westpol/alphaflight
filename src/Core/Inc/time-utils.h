/*
 * time-utils.h
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#ifndef INC_TIME_UTILS_H_
#define INC_TIME_UTILS_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx);

uint64_t MICROS64();
uint64_t MICROS64_USB();
TIM_HandleTypeDef* TIME_UTILS_GET_TIMER();
void TIME_UTILS_PAUSE_MICROS64();
void TIME_UTILS_CONTINUE_MICROS64();

#endif /* INC_TIME_UTILS_H_ */
