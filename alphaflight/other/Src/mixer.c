#include "mixer.h"
#include "servo.h"

MIXER_RETURN_TYPE MIXER_SET_POSITION(uint16_t elevator, uint16_t aileron, uint16_t rudder){
    SERVO_SET_PWM(0, aileron);
    SERVO_SET_PWM(1, aileron);
    SERVO_SET_PWM(2, elevator);
    SERVO_SET_PWM(3, rudder);

    return MIXER_OKAY;
}