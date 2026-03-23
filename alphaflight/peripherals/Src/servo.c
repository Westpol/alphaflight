#include "servo.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_tim.h"

static servo_config_t config = {0};
static bool config_set = false;
static bool initialized = false;

SERVO_RETURN_TYPE SERVO_INIT(){
    if(!config_set) return SERVO_FAIL;
    if(initialized) return SERVO_FAIL;

    TIM_TypeDef* servo_1_4 = TIM2;  // servo timers in vars for easier change if hardware ever changes
    TIM_TypeDef* servo_5_8 = TIM3;

    servo_1_4->CCR1 = config.servo_idle_pos[0]; // load idle vals channels 1-4
    servo_1_4->CCR2 = config.servo_idle_pos[1];
    servo_1_4->CCR3 = config.servo_idle_pos[2];
    servo_1_4->CCR4 = config.servo_idle_pos[3];

    servo_5_8->CCR1 = config.servo_idle_pos[4]; // load idle vals channels 5-8
    servo_5_8->CCR2 = config.servo_idle_pos[5];
    servo_5_8->CCR3 = config.servo_idle_pos[6];
    servo_5_8->CCR4 = config.servo_idle_pos[7];

    servo_1_4->CCER |= LL_TIM_CHANNEL_CH1;  // enable channels 1-4
    servo_1_4->CCER |= LL_TIM_CHANNEL_CH2;
    servo_1_4->CCER |= LL_TIM_CHANNEL_CH3;
    servo_1_4->CCER |= LL_TIM_CHANNEL_CH4;

    servo_5_8->CCER |= LL_TIM_CHANNEL_CH1;  // enable channels 5-8
    servo_5_8->CCER |= LL_TIM_CHANNEL_CH2;
    servo_5_8->CCER |= LL_TIM_CHANNEL_CH3;
    servo_5_8->CCER |= LL_TIM_CHANNEL_CH4;

    servo_1_4->CR1 |= TIM_CR1_CEN;  // enable timers
    servo_5_8->CR1 |= TIM_CR1_CEN;

    initialized = true;
    return SERVO_OKAY;
}

SERVO_RETURN_TYPE SERVO_SET_CONFIG(servo_config_t new_config){
    config = new_config;
    config_set = true;  // make sure that config is loaded before servos are initialized
    return SERVO_OKAY;
}

servo_config_t SERVO_GET_DEFAULT_CONFIG(){
    servo_config_t temp = {0};
    for(uint8_t i = 0; i < 8; i++){
        temp.servo_direction[i] = 1;
        temp.servo_neutral_pos[i] = 1501;
        temp.servo_function_map[i] = 1;
        temp.servo_idle_pos[i] = 1500;
        temp.servo_max[i] = 2000;
        temp.servo_min[i] = 1000;
        temp.servo_max_delta_per_update[i] = 1000;
    }
    return temp;
}