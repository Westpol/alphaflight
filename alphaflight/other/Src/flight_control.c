#include "flight_control.h"
#include "crossfire.h"
#include "dshot.h"
#include "servo.h"
#include "status_led.h"
#include "logger.h"
#include "mixer.h"

static bool logging = false;
static bool armed = false;
static bool arming_failed = false;

uint32_t FC_DIRECT_LAW(const task_info_t* task){
    CRSF_CHANNELS_T crsf = CRSF_GET_CHANNELS_DATA();

    // arming logic
    if(crsf.channel[4] > 1800){
        if(!armed){
            if(!armed && !arming_failed && crsf.channel_raw[0] - 173 == 0){
                armed = true;
            }
            else {
                arming_failed = true;
            }
        }
    }

    if(crsf.channel[4] < 1800){
        arming_failed = false;
        armed = false;
    }

    // motor command
    if(armed && !arming_failed){  // Motor armed
        DSHOT_SET_THROTTLE((crsf.channel_raw[0] - 173), true);
    }
    else {  // Motor disarmed
        DSHOT_SET_THROTTLE(0, false);
    }



    // servo command
    if(crsf.channel[4] > 1100){
        MIXER_SET_POSITION(crsf.channel[1], crsf.channel[2]);
    }

    // logging logic
    if(crsf.channel[4] > 1100 && logging == false){
        LOG_START();
        logging = true;
        STATUS_LED_SET_R(1000);
    }
    if(crsf.channel[4] < 1100 && logging == true){
        LOG_END();
        logging = false;
        STATUS_LED_SET_R(0);
    }

    return 0;
}