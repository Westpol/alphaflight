#include "common.h"

#ifndef SERVO_H
#define SERVO_H

typedef enum{
    SERVO_OKAY,
    SERVO_FAIL
}SERVO_RETURN_TYPE;

SERVO_RETURN_TYPE SERVO_INIT();

#endif