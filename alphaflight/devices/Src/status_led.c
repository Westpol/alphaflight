#include "status_led.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_tim.h"
#include "stdlib.h"

#define TIMER_CEN_MASK 0x1

void STATUS_LED_INIT(void){
    TIM4->CCER |= LL_TIM_CHANNEL_CH2;   // enable channels
    TIM4->CCER |= LL_TIM_CHANNEL_CH3;
    TIM4->CCER |= LL_TIM_CHANNEL_CH4;
    TIM4->CR1 |= 0x01 & TIMER_CEN_MASK;     // enable timer
}

void STATUS_LED_SET_ALL(uint32_t brightness){
    TIM4->CCR2 = brightness;
    TIM4->CCR3 = brightness;
    TIM4->CCR4 = brightness;
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

uint32_t rgb[3] = {1, 0, 0};
int8_t changing_index = 0;
bool sign = true;
uint32_t counter = 0;

uint32_t STATUS_PULSE(const task_info_t* info){
    if(rgb[changing_index] >= 100) sign = false;

    if(rgb[changing_index] <= 0){
        changing_index = (changing_index + 1) % 3;
        rgb[changing_index] = 1;
        sign = true;
    }
    STATUS_LED_SET_R(rgb[0]);
    STATUS_LED_SET_G(rgb[1]);
    STATUS_LED_SET_B(rgb[2]);
    if(sign) rgb[changing_index]++;
    else rgb[changing_index]--;
    return 1000;
}