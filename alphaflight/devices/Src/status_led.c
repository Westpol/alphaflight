#include "status_led.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_tim.h"


void STATUS_LED_INIT(void){
    TIM4->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;   // enable channels
    TIM4->CR1 |= TIM_CR1_CEN;   // enable counter
}

void STATUS_LED_SET_ALL(uint32_t brightness){
    TIM4->CCR2 = brightness;    // set Compare registers to brightness
    TIM4->CCR3 = brightness;
    TIM4->CCR4 = brightness;
}

void STATUS_LED_SET_R(uint32_t brightness){
    TIM4->CCR4 = brightness;    // set Compare registers to brightness
}

void STATUS_LED_SET_G(uint32_t brightness){
    TIM4->CCR3 = brightness;    // set Compare registers to brightness
}

void STATUS_LED_SET_B(uint32_t brightness){
    TIM4->CCR2 = brightness;    // set Compare registers to brightness
}