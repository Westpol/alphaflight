#include "flight_control.h"
#include "crossfire.h"
#include "dshot.h"
#include "servo.h"
#include "status_led.h"
#include "logger.h"

static bool logging = false;

uint32_t FC_DIRECT_LAW(const task_info_t* task){
    CRSF_CHANNELS_T crsf = CRSF_GET_CHANNELS();
    if(crsf.channel[4] > 1500){  // Motor armed
        DSHOT_SET_THROTTLE((crsf.channel[0] - 172) + 1, true);
    }
    else {  // Motor disarmed
        DSHOT_SET_THROTTLE(0, false);
    }
    if(crsf.channel[11] > 1000 && logging == false){
        LOG_START();
        logging = true;
        STATUS_LED_SET_R(1000);
    }
    if(crsf.channel[11] < 1000 && logging == true){
        LOG_END();
        logging = false;
        STATUS_LED_SET_R(0);
    }
    return 0;
}