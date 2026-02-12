#include "common.h"
#include "spi.h"
#include "scheduler.h"

#ifndef LSM6DSO_H
#define LSM6DSO_H

typedef enum{
    IMU_OKAY,
    IMU_WRONG_ID,
    IMU_FAIL
} IMU_RETURN_TYPE;

uint32_t IMU_CONVERT_DATA(const task_info_t *task);

void IMU_DATA_READY_INTERRUPT_HANDLER(void);
void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void);

#endif