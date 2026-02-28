#include "status_led.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_tim.h"
#include "stdlib.h"

#define TIMER_CEN_MASK 0x1

uint32_t phase = 0;

void STATUS_LED_INIT(void){
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
    TIM4->CR1 |= 0x01 & TIMER_CEN_MASK;
}

void STATUS_LED_SET_R(uint32_t brightness){
    TIM4->CCR4 = brightness;
}

void STATUS_LED_SET_G(uint32_t brightness){
    TIM4->CCR3 = brightness;
}

void STATUS_LED_SET_B(uint32_t brightness){
    TIM4->CCR2 = brightness;
}

uint32_t STATUS_PULSE(const task_info_t* info){
    phase = (phase + 1) % 2000;
    uint32_t brightness = abs((int32_t)phase - 1000);

    STATUS_LED_SET_R(brightness);
    STATUS_LED_SET_G(brightness);
    STATUS_LED_SET_B(brightness);
    return 1000;
}