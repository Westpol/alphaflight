#include "flight_control.h"
#include "crossfire.h"
#include "dshot.h"
#include "servo.h"

uint32_t FC_DIRECT_LAW(const task_info_t* task){
    CRSF_CHANNELS_T crsf = CRSF_GET_CHANNELS();
    if(crsf.channel[4] > 1500){  // Motor armed
        DSHOT_SET_THROTTLE((crsf.channel[0] - 172) + 1);
    }
    else {  // Motor disarmed
        DSHOT_SET_THROTTLE(0);
    }
    return 0;
}