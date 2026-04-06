#include "common.h"
#include "spi.h"
#include "scheduler.h"
#include "stm32h7xx_hal.h"

#ifndef LSM6DSO_H
#define LSM6DSO_H

#define LSM6DSO_INTERRUPT true   // switch between fixed interval blocked SPI polling (true) and DRDY based DMA SPI (false)

typedef enum{
    IMU_OKAY,
    IMU_WRONG_ID,
    IMU_SETUP_FAILED,
    IMU_FAIL
} IMU_RETURN_TYPE;

typedef struct{
    uint8_t orientation;
}imu_config_t;

typedef struct{
    float w;
    float x;
    float y;
    float z;
}quat_t;

typedef struct{
    float wx;
    float wy;
    float wz;
}rate_t;

typedef struct{
    float x;
    float y;
    float z;
}accel_t;

typedef struct{
    int16_t wx;
    int16_t wy;
    int16_t wz;
}rate_raw_t;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
}accel_raw_t;

typedef struct{
    quat_t quat;
    rate_t rate;
    accel_t accel;
}IMU_PROCESSED_T;

typedef struct{
    rate_raw_t rate_raw;
    accel_raw_t accel_raw;
}imu_raw_t;

typedef struct{
    IMU_PROCESSED_T processed;
    imu_raw_t raw;
}IMU_T;

IMU_RETURN_TYPE IMU_INIT(SPI_DEVICE device, int32_t gyro_convert_task_index, DMA_HandleTypeDef* spi_rx_dma);

#if LSM6DSO_POLLING
    uint32_t IMU_CONVERT_DATA(const task_info_t *task);
#endif

#if !LSM6DSO_POLLING
    uint32_t IMU_READ_DATA(const task_info_t* task);
#endif

void IMU_DATA_READY_INTERRUPT_HANDLER(void);
void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void);

IMU_PROCESSED_T IMU_GET_DATA();

void IMU_SET_CONFIG(imu_config_t new_config);
imu_config_t IMU_GET_DEFAULT_CONFIG();

#define LSM6DSO_WRITE 0x7F
#define LSM6DSO_READ 0x80

#define LSM6DSO_INT1_CTRL_ADDRESS 0x0D
#define LSM6DSO_INT1_CTRL_DRDY_G 0x02

#define LSM6DSO_CTRL1_XL_ADDRESS 0x10
#define LSM6DSO_CTRL1_XL_ODR_1666 (0x08 << 4)   // CORRECTLY SET CTLR6_C
#define LSM6DSO_CTRL1_XL_FS_16 (0x01 << 2)      // CORRECTLY SET CTLR8_XL
#define LSM6DSO_CTRL1_XL_MASK_AND 0xFE

#define LSM6DSO_CTRL2_G_ADDRESS 0x11
#define LSM6DSO_CTRL2_G_ODR_1666 (0x08 << 4)    // CORRECTLY SET CTRL7_G
#define LSM6DSO_CTRL2_G_FS_2500 (0x03 << 2)
#define LSM6DSO_CTRL2_G_MASK_AND 0xFE

#define LSM6DSO_CRTL3_C_ADDRESS 0x12
#define LSM6DSO_CTRL3_C_RESET (0x01)  // resets IMU

#define LSM6DSO_CRTL4_C_ADDRESS 0x13
#define LSM6DSO_CTRL4_C_DRDY_MASK (0x01 << 3)   // only set DRDY once filters have setteled
#define LSM6DSO_CTRL4_C_MASK_AND 0b01101110

#define LSM6DSO_CTRL9_XL_ADDRESS 0x19
#define LSM6DSO_CTRL9_XL_I3C_DISABLE (0x01 << 1)

#define LSM6DSO_READ_START_REG 0x22

#endif