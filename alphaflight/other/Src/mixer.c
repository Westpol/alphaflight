#include "mixer.h"
#include "servo.h"

MIXER_RETURN_TYPE MIXER_SET_POSITION(uint16_t elevator, uint16_t aileron){
    SERVO_SET_PWM(0, aileron);
    SERVO_SET_PWM(1, aileron);
    SERVO_SET_PWM(2, elevator);

    return MIXER_OKAY;
}