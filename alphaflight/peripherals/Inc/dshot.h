#include "common.h"
#include "scheduler.h"
#include <stdint.h>


#ifndef DSHOT_H
#define DSHOT_H

#define DSHOT_ENABLE_IDLE_SPINNING false

typedef enum{
    DSHOT_OKAY,
    DSHOT_FAIL
}DSHOT_RETURN_TYPE;

typedef struct{
    uint16_t motor_idle;
    uint16_t motor_max;
}dshot_config_t;

DSHOT_RETURN_TYPE DSHOT_INIT(void);

uint32_t DSHOT_TRANSMIT(const task_info_t* task);
DSHOT_RETURN_TYPE DSHOT_SET_THROTTLE(uint16_t throttle, bool armed);

DSHOT_RETURN_TYPE DSHOT_DMA_CPLT_CALLBACK(void);

DSHOT_RETURN_TYPE DSHOT_SET_CONFIG(dshot_config_t conf);
dshot_config_t DSHOT_GET_DEFAULT_CONFIG(void);

#endif