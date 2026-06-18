#include "power_measurement.h"
#include "usb.h"

#define ADC_MAX 16384

#define VBAT_SHIFT 14
#define CURR_SHIFT 14

uint32_t last_conversion_time = 0;

__attribute__((section(".dma_rx"))) static uint32_t adc_buffer[2] = {0};
static ADC_HandleTypeDef* adc_handle_global = NULL;

POWER_DATA_T data = {0};
static power_config_t config = {0};
static bool config_set = false;

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_INIT(ADC_HandleTypeDef* adc_handle){
    // ADC should already be initialized, just start it in DMA mode
    if(adc_handle == NULL) return POWER_MEASUREMENT_FAIL;
    adc_handle_global = adc_handle;
    return POWER_MEASUREMENT_OKAY;
}

uint32_t POWER_MEASUREMENT_START_DMA_READ(const task_info_t* task){
    if(adc_handle_global == NULL) return 0;
    if(HAL_ADC_Start_DMA(adc_handle_global, adc_buffer, 2) != HAL_OK) return 0;
    return 0;
}


#define us_to_hours ((1.0f / 1000000.0f) / 3600.0f)
uint32_t POWER_MEASUREMENT_CONVERT(const task_info_t* task){
    uint32_t now = MICROS32();
    uint32_t delta = now - last_conversion_time;
    last_conversion_time = now;
    data.voltage = (adc_buffer[0] * config.voltage_scale_mV) >> VBAT_SHIFT;   // in mV, assuming 3.3V reference and 14 bit ADC
    data.current = (adc_buffer[1] * config.current_scale_mA) >> CURR_SHIFT;   // in mA, assuming 104A max current and 14 bit ADC
    data.voltage += config.voltage_offset_mV;
    data.current += config.current_offset_mA;
    data.voltage = data.voltage > 40000 ? 0 : data.voltage;   // filter out invalid readings above 40V
    data.current = data.current > 100000 ? 0 : data.current;   // filter out invalid readings above 100A
    data.capacity_used_mAh += (float)data.current * us_to_hours * delta;
    return 0;
}

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_PRINT_DATA(void){
    USB_PRINTLN("VBat: %f V\nCurrent: %f A\nBattery used (mAh): %d mAh\n Battery Capacity: %d mAh\n Battery capacity used: %d percent", data.voltage / 1000.0f, data.current / 1000.0f, (uint32_t)data.capacity_used_mAh, config.battery_capacity_mAh, (uint32_t)((data.capacity_used_mAh / config.battery_capacity_mAh) * 100));
    return POWER_MEASUREMENT_OKAY;
}


POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_DMA_CALLBACK(void){
    SCHEDULER_REGISTER_THROWAWAY_TASK(POWER_MEASUREMENT_CONVERT, 100, "Power Measurement Conversion");
    return POWER_MEASUREMENT_OKAY;
}

POWER_DATA_T POWER_GET_DATA(void){
    return data;
}

POWER_MEASUREMENT_RETURN_TYPE POWER_MEASUREMENT_SET_CONFIG(power_config_t new_config){
    config = new_config;
    config_set = true;  // make sure that config is loaded before servos are initialized
    return POWER_MEASUREMENT_OKAY;
}


power_config_t POWER_MEASUREMENT_GET_CONFIG(void){
    return config;
}


power_config_t POWER_MEASUREMENT_GET_DEFAULT_CONFIG(void){
    power_config_t temp = {0};
    temp.voltage_scale_mV = 33000u;
    temp.current_scale_mA = 104000u;
    temp.voltage_offset_mV = 1150u;
    temp.current_offset_mA = -2400u;
    temp.battery_capacity_mAh = 3000u;
    return temp;
}