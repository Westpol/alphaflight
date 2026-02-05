#include "stm32h723xx.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef TIMER_H_
#define TIMER_H_

typedef enum{
    TIMER_INIT_OKAY,
    TIMER_INIT_FAULT,
    TIMER_INIT_ALREADY_DONE,
    TIMER_OKAY,
    TIMER_START_ALREADY_STARTED,
    TIMER_START_NOT_INITIALIZED,
    TIMER_STOP_ALREADY_STOPPED,
    TIMER_STOP_NOT_INITIALIZED
} TIMER_RETURN_TYPE;

TIMER_RETURN_TYPE TIMER_INIT(TIM_TypeDef* millis_timer, TIM_TypeDef* micros_timer);

TIMER_RETURN_TYPE TIMER_START(void);

TIMER_RETURN_TYPE TIMER_STOP(void);

bool TIMER_RUNNING(void);

uint32_t MICROS32(void);

uint32_t MILLIS32(void);

#endif