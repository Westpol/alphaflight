#include "common.h"
#include "stm32h7xx_hal.h"
#include "scheduler.h"

#ifndef POWER_MEASUREMENT_H
#define POWER_MEASUREMENT_H

typedef struct{
    uint32_t voltage_scale_mV;   // scale factor for converting ADC reading to mV
    uint32_t current_scale_mA;   // scale factor for converting ADC reading to mA
    int32_t voltage_offset_mV;  // offset to add to voltage reading in mV
    int32_t current_offset_mA;  // offset to add to current reading in mA
} power_config_t;

typedef struct{
    uint32_t voltage;   // in mV
    uint32_t current;   // in mA
} POWER_DATA_T;

typedef enum{
    POWER_MEASUREMENT_OKAY,
    POWER_MEASUREMENT_FAIL
} POWER_MEASUREMENT_RETURN_TYPE;

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_INIT(ADC_HandleTypeDef* adc_handle);

uint32_t POWER_MEASUREMENT_START_DMA_READ(const task_info_t* task);
uint32_t POWER_MEASUREMENT_CONVERT(const task_info_t* task);

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_DMA_CALLBACK(void);

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_SET_CONFIG(power_config_t new_config);
power_config_t POWER_MEASUREMENT_GET_CONFIG(void);
power_config_t POWER_MEASUREMENT_GET_DEFAULT_CONFIG(void);

#endif