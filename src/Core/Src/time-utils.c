/*
 * time-utils.c
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#include "time-utils.h"
#include <stdbool.h>

TIM_HandleTypeDef *htim = NULL;
static volatile uint32_t timer_high = 0;
static volatile uint32_t last_timer_read = 0;

static volatile uint32_t timer_high_usb = 0;
static volatile uint32_t last_timer_read_usb = 0;

static volatile uint32_t timer_paused_state = 0;
static volatile bool initialized = false;
static volatile bool initialized_usb = false;

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx) {
	htim = HTIMx;
	HAL_TIM_Base_Start(htim);
	initialized = true;
}

TIM_HandleTypeDef* TIME_UTILS_GET_TIMER(){
	return htim;
}

void TIME_UTILS_PAUSE_MICROS64(){
	initialized = false;
	initialized_usb = false;
	HAL_TIM_Base_Stop(htim);
	__HAL_TIM_DISABLE(htim);
	timer_paused_state = htim->Instance->CNT;
	htim->Instance->CNT = 0;
	timer_high_usb = 0;
	last_timer_read_usb = 0;

	__HAL_TIM_ENABLE(htim);
	HAL_TIM_Base_Start(htim);
	initialized_usb = true;
}

void TIME_UTILS_CONTINUE_MICROS64(){
	initialized = false;
	initialized_usb = false;
	HAL_TIM_Base_Stop(htim);
	__HAL_TIM_DISABLE(htim);
	htim->Instance->CNT = timer_paused_state;
	__HAL_TIM_ENABLE(htim);
	HAL_TIM_Base_Start(htim);
	initialized = true;
}

uint64_t MICROS64(void) {
	if(initialized){
		uint32_t timer_low   = htim->Instance->CNT;
		if(timer_low < last_timer_read){		// overflow occured
			timer_high += 1;
		}
		last_timer_read = timer_low;

		return (((uint64_t)timer_high << 32) | timer_low);
	}
	return (((uint64_t)timer_high << 32) | timer_paused_state);
}

uint64_t MICROS64_USB(void) {
	if(initialized_usb){
		uint32_t timer_low   = htim->Instance->CNT;
		if(timer_low < last_timer_read_usb){		// overflow occured
			timer_high_usb += 1;
		}
		last_timer_read_usb = timer_low;

		return (((uint64_t)timer_high_usb << 32) | timer_low);	// returns frozen timestamp
	}
	return 0;
}
