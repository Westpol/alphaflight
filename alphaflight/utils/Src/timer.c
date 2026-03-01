#include "timer.h"
#include "stm32h723xx.h"
#include <stdint.h>
#include <stddef.h>

static TIM_TypeDef* millis_timer = NULL;
static TIM_TypeDef* micros_timer = NULL;

#define TIMER_CEN_MASK 0x1

TIMER_RETURN_TYPE TIMER_INIT(TIM_TypeDef* timer_millis, TIM_TypeDef* timer_micros){
    if(timer_millis == NULL || timer_micros == NULL) return TIMER_INIT_FAULT;
    if(millis_timer != NULL || micros_timer != NULL) return TIMER_INIT_ALREADY_DONE;

    millis_timer = timer_millis;
    micros_timer = timer_micros;

    return TIMER_INIT_OKAY;
}

TIMER_RETURN_TYPE TIMER_START(){
    if(millis_timer == NULL || micros_timer == NULL) return TIMER_START_NOT_INITIALIZED;

    if(millis_timer->CR1 & TIMER_CEN_MASK || micros_timer->CR1 & TIMER_CEN_MASK) return TIMER_START_ALREADY_STARTED;

    millis_timer->CR1 |= 0x01 & TIMER_CEN_MASK;     // activate timers, change nothing else
    micros_timer->CR1 |= 0x01 & TIMER_CEN_MASK;

    return TIMER_OKAY;
}

TIMER_RETURN_TYPE TIMER_STOP(){
    if(millis_timer == NULL || micros_timer == NULL) return TIMER_START_NOT_INITIALIZED;

    if(!(millis_timer->CR1 & TIMER_CEN_MASK) || !(micros_timer->CR1 & TIMER_CEN_MASK)) return TIMER_STOP_ALREADY_STOPPED;

    millis_timer->CR1 &= ~TIMER_CEN_MASK;        // deactivate timers, change nothing else
    micros_timer->CR1 &= ~TIMER_CEN_MASK;

    return TIMER_OKAY;
}

uint32_t MICROS32(){
    return micros_timer ? micros_timer->CNT : 0;
}

uint32_t MILLIS32(){
    return millis_timer ? millis_timer->CNT >> 1 : 0;
}

bool TIMER_RUNNING(){
    if(millis_timer == NULL || micros_timer == NULL) return false;

    if(!(millis_timer->CR1 & TIMER_CEN_MASK) || !(micros_timer->CR1 & TIMER_CEN_MASK)) return false;

    return true;
}