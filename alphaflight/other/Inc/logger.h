#include "common.h"
#include "scheduler.h"

#ifndef LOGGER_H
#define LOGGER_H

typedef enum{
    LOG_OKAY,
    LOG_FAIL
} LOG_RETURN_TYPE;

typedef struct{
    uint32_t log_frequency;
    uint32_t gps_log_divider;
    uint32_t imu_log_divider;
    uint32_t baro_log_divider;
    uint32_t crsf_log_divider;
    bool delta_mode;
} log_config_t;

LOG_RETURN_TYPE LOG_START();
uint32_t LOG_RUN(const task_info_t* task);
LOG_RETURN_TYPE LOG_END();

void LOG_SET_CONFIG(log_config_t new_config);
log_config_t LOG_GET_DEFAULT_CONFIG();

void LOG_SET_TASK_INDEX(int32_t index);

uint32_t LOG_GET_FREQUENCY(void);

#endif