#include "servo.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_tim.h"
#include <stdint.h>
#include <string.h>

static servo_config_t config = {0};
static bool config_set = false;
static bool initialized = false;

TIM_TypeDef* servo_timers[2] = {NULL, NULL};

SERVO_RETURN_TYPE SERVO_INIT(TIM_TypeDef* servo_0_3, TIM_TypeDef* servo_4_7){
    if(!config_set) return SERVO_FAIL;
    if(initialized) return SERVO_FAIL;

    servo_timers[0] = servo_0_3;
    servo_timers[1] = servo_4_7;

    servo_timers[0]->CCR1 = config.servo_idle_pos[0]; // load idle vals channels 1-4
    servo_timers[0]->CCR2 = config.servo_idle_pos[1];
    servo_timers[0]->CCR3 = config.servo_idle_pos[2];
    servo_timers[0]->CCR4 = config.servo_idle_pos[3];

    servo_timers[1]->CCR1 = config.servo_idle_pos[4]; // load idle vals channels 5-8
    servo_timers[1]->CCR2 = config.servo_idle_pos[5];
    servo_timers[1]->CCR3 = config.servo_idle_pos[6];
    servo_timers[1]->CCR4 = config.servo_idle_pos[7];

    servo_timers[0]->CCER |= LL_TIM_CHANNEL_CH1;  // enable channels 1-4
    servo_timers[0]->CCER |= LL_TIM_CHANNEL_CH2;
    servo_timers[0]->CCER |= LL_TIM_CHANNEL_CH3;
    servo_timers[0]->CCER |= LL_TIM_CHANNEL_CH4;

    servo_timers[1]->CCER |= LL_TIM_CHANNEL_CH1;  // enable channels 5-8
    servo_timers[1]->CCER |= LL_TIM_CHANNEL_CH2;
    servo_timers[1]->CCER |= LL_TIM_CHANNEL_CH3;
    servo_timers[1]->CCER |= LL_TIM_CHANNEL_CH4;

    servo_timers[0]->CR1 |= TIM_CR1_CEN;  // enable timers
    servo_timers[1]->CR1 |= TIM_CR1_CEN;

    initialized = true;
    return SERVO_OKAY;
}

SERVO_RETURN_TYPE SERVO_SET_PWM(uint8_t servo, uint16_t pwm_val){
    pwm_val = UTILS_MIN_MAX_I(pwm_val, 1000, 2000);
    if(servo <= 3){
        ((volatile uint32_t*)(&servo_timers[0]->CCR1))[servo] = pwm_val;
    }
    else if (servo <= 7) {
        ((volatile uint32_t*)(&servo_timers[1]->CCR1))[servo - 4] = pwm_val;
    }
    else{
        return SERVO_FAIL;
    }
    return SERVO_OKAY;
}

SERVO_RETURN_TYPE SERVO_SET_CONFIG(servo_config_t new_config){
    config = new_config;
    config_set = true;  // make sure that config is loaded before servos are initialized
    return SERVO_OKAY;
}


servo_config_t SERVO_GET_CONFIG(void){
    return config;
}


servo_config_t SERVO_GET_DEFAULT_CONFIG(void){
    servo_config_t temp = {0};
    for(uint8_t i = 0; i < 8; i++){
        temp.servo_direction[i] = 1;
        temp.servo_neutral_pos[i] = 1500;
        temp.servo_function_map[i] = 1;
        temp.servo_idle_pos[i] = 1500;
        temp.servo_max[i] = 2000;
        temp.servo_min[i] = 1000;
        temp.servo_max_delta_per_update[i] = 1000;
    }
    return temp;
}