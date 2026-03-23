#include "common.h"

#ifndef SERVO_H
#define SERVO_H

typedef enum{
    SERVO_OKAY,
    SERVO_FAIL
}SERVO_RETURN_TYPE;

typedef enum{
    SERVO_AILERON_LEFT,
    SERVO_AILERON_RIGHT,
    SERVO_RUDDER,
    SERVO_ELEVATOR
}SERVO_ROLES;

typedef struct{
    uint32_t servo_idle_pos[8];     // start and rxloss position
    uint32_t servo_neutral_pos[8];  // neutral position for rudders
    uint32_t servo_max[8];      // max servo angle (angle at 100%)
    uint32_t servo_min[8];      // min servo angle (angle at -100%)
    int8_t servo_direction[8];
    uint32_t servo_max_delta_per_update[8]; // prevents mechanical stress
    uint8_t servo_function_map[8];
}servo_config_t;

SERVO_RETURN_TYPE SERVO_INIT();

SERVO_RETURN_TYPE SERVO_SET_CONFIG(servo_config_t new_config);
servo_config_t SERVO_GET_DEFAULT_CONFIG();

#endif