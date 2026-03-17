#include "common.h"
#include "scheduler.h"
#include "spi.h"
#include <stdint.h>

#ifndef BMP390_H
#define BMP390_H

typedef enum{
    BARO_OKAY,
    BARO_WRONG_ID,
    BARO_FAIL
} BARO_RETURN_TYPE;

typedef struct{
    uint32_t temp;
    uint32_t pressure;
}baro_data_raw_t;

typedef struct{
    float pressure_base;
}baro_data_processing_t;

typedef struct{
    float temp;
    float pressure;
    float height;
}BARO_PROCESSED_T;

typedef struct{
    baro_data_raw_t raw;
    baro_data_processing_t processing;
    BARO_PROCESSED_T processed;
}BARO_T;

BARO_RETURN_TYPE BARO_INIT(SPI_DEVICE device);

uint32_t BARO_READ_DATA(const task_info_t* task);

BARO_PROCESSED_T BARO_GET_DATA();

typedef struct {
	uint16_t NVM_PAR_T1;
	uint16_t NVM_PAR_T2;
	int8_t NVM_PAR_T3;
	int16_t NVM_PAR_P1;
	int16_t NVM_PAR_P2;
	int8_t NVM_PAR_P3;
	int8_t NVM_PAR_P4;
	uint16_t NVM_PAR_P5;
	uint16_t NVM_PAR_P6;
	int8_t NVM_PAR_P7;
	int8_t NVM_PAR_P8;
	int16_t NVM_PAR_P9;
	int8_t NVM_PAR_P10;
	int8_t NVM_PAR_P11;
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} Baro_Calibration;

#define BMP390_READ 0x80
#define BMP390_WRITE 0x7F

#define BMP390_DATA_START_ADDRESS 0x04

#define BMP390_ID_ADDRESS 0x00
#define BMP390_ID_VALUE 0x60

#define BMP390_ERR_ADDRESS 0x02

#define BMP390_STAT_ADDRESS 0x03

#define BMP390_PWR_ADDRESS 0x1B
#define BMP390_PWR_PRESS_ON 0x01
#define BMP390_PWR_TEMP_ON (0x01 << 1)
#define BMP390_PWR_MODE_NORMAL (0x03 << 4)

#define BMP390_OSR_ADDRESS 0x1C
#define BMP390_OSR_PRESS_X16 0x04
#define BMP390_OSR_TEMP_X2 (0x01 << 3)

#define BMP390_ODR_ADDRESS 0x1D
#define BMP390_ODR_25HZ 0x03

#define BMP390_IIR_ADDRESS 0x1F
#define BMP390_IIR_COEF_3 (0x02 << 1)
#define BMP390_IIR_COEF_7 (0x03 << 1)
#define BMP390_IIR_COEF_15 (0x04 << 1)

#define BMP390_CMD_ADDRESS 0x7E
#define BMP390_CMD_RESET 0xB6

#endif