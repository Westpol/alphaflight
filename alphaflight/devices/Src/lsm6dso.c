#include "lsm6dso.h"
#include "main.h"
#include "timer.h"

#define imu_execution_delta_offset 20       // offset in us
#define imu_execution_averaging_bias 9

static IMU_T imu = {0};

static SPI_DEVICE spi_device;
DMA_HandleTypeDef* rx_dma = NULL;

static imu_config_t config = {0};

static uint32_t last_integration = 0;

static int32_t imu_convert_task_index = -1;
volatile static bool imu_set_up = false;

float twoKp = 2.0f * 1.0f;   // proportional gain (tune this!)
float twoKi = 2.0f * 0.0f;   // integral gain (start with 0)
float integralFB[3] = {0};

#if LSM6DSO_INTERRUPT
__attribute__((section(".dma_rx"))) static uint8_t imu_dma_rx[STM32_WORD_SIZE] = {0};
__attribute__((section(".dma_tx"))) static uint8_t imu_dma_tx[STM32_WORD_SIZE] = {0};
#endif

static IMU_RETURN_TYPE imu_convert_data();
static uint8_t read_register(uint8_t address);
static IMU_RETURN_TYPE write_register(uint8_t address, uint8_t data);
static IMU_RETURN_TYPE imu_setup();
static IMU_RETURN_TYPE imu_update_quat(void);


static IMU_RETURN_TYPE imu_setup(){
    write_register(LSM6DSO_CRTL3_C_ADDRESS, LSM6DSO_CTRL3_C_RESET);
    while((read_register(LSM6DSO_CRTL3_C_ADDRESS) & 0x01));
    write_register(LSM6DSO_CTRL9_XL_ADDRESS, LSM6DSO_CTRL9_XL_I3C_DISABLE);

    switch (config.odr) {
        case IMU_ODR_833Hz:
            write_register(LSM6DSO_CTRL1_XL_ADDRESS, (LSM6DSO_CTRL1_XL_ODR_833 | LSM6DSO_CTRL1_XL_FS_16) & LSM6DSO_CTRL1_XL_MASK_AND);
            write_register(LSM6DSO_CTRL2_G_ADDRESS, (LSM6DSO_CTRL2_G_ODR_833 | LSM6DSO_CTRL2_G_FS_2500) & LSM6DSO_CTRL2_G_MASK_AND);
        break;
        case IMU_ODR_1666Hz:
            write_register(LSM6DSO_CTRL1_XL_ADDRESS, (LSM6DSO_CTRL1_XL_ODR_1666 | LSM6DSO_CTRL1_XL_FS_16) & LSM6DSO_CTRL1_XL_MASK_AND);
            write_register(LSM6DSO_CTRL2_G_ADDRESS, (LSM6DSO_CTRL2_G_ODR_1666 | LSM6DSO_CTRL2_G_FS_2500) & LSM6DSO_CTRL2_G_MASK_AND);
        break;
        case IMU_ODR_3333Hz:
            write_register(LSM6DSO_CTRL1_XL_ADDRESS, (LSM6DSO_CTRL1_XL_ODR_3333 | LSM6DSO_CTRL1_XL_FS_16) & LSM6DSO_CTRL1_XL_MASK_AND);
            write_register(LSM6DSO_CTRL2_G_ADDRESS, (LSM6DSO_CTRL2_G_ODR_3333 | LSM6DSO_CTRL2_G_FS_2500) & LSM6DSO_CTRL2_G_MASK_AND);
        break;
        case IMU_ODR_6666Hz:
            write_register(LSM6DSO_CTRL1_XL_ADDRESS, (LSM6DSO_CTRL1_XL_ODR_6666 | LSM6DSO_CTRL1_XL_FS_16) & LSM6DSO_CTRL1_XL_MASK_AND);
            write_register(LSM6DSO_CTRL2_G_ADDRESS, (LSM6DSO_CTRL2_G_ODR_6666 | LSM6DSO_CTRL2_G_FS_2500) & LSM6DSO_CTRL2_G_MASK_AND);
        break;
    }

    write_register(LSM6DSO_CRTL4_C_ADDRESS, LSM6DSO_CTRL4_C_DRDY_MASK & LSM6DSO_CTRL4_C_MASK_AND);
    #if LSM6DSO_INTERRUPT
        write_register(LSM6DSO_INT1_CTRL_ADDRESS, LSM6DSO_INT1_CTRL_DRDY_G);
        write_register(LSM6DSO_COUNTER_BDR_REG1_ADDRESS, LSM6DSO_COUNTER_BDR_REG1_DRDY_PULSED);
    #endif
    imu_set_up = true;
    return IMU_OKAY;
}


IMU_RETURN_TYPE IMU_INIT(SPI_DEVICE device, int32_t gyro_convert_task_index, DMA_HandleTypeDef* spi_rx_dma){
    spi_device = device;
    rx_dma = spi_rx_dma;
    imu_convert_task_index = gyro_convert_task_index;
    imu.processed.quat.w = 1.0f;    // set to standard orientation
    imu_dma_tx[0] = LSM6DSO_READ_START_REG | LSM6DSO_READ;
    if(read_register(0x8F) != 108) return IMU_WRONG_ID; // check if IMU is registered

    if(imu_setup() != IMU_OKAY) return IMU_SETUP_FAILED;

    #if LSM6DSO_INTERRUPT

    __HAL_DMA_ENABLE_IT(spi_rx_dma, DMA_IT_TC);

    #endif

    return IMU_OKAY;
}

static IMU_RETURN_TYPE imu_update_quat(void){
    float dt = (float)(MICROS32() - last_integration) / 1000000.0f;
    last_integration = MICROS32();

    float q0 = imu.processed.quat.w;
    float q1 = imu.processed.quat.x;
    float q2 = imu.processed.quat.y;
    float q3 = imu.processed.quat.z;

    float gx = imu.processed.rate.wx;
    float gy = imu.processed.rate.wy;
    float gz = imu.processed.rate.wz;

    float ax = imu.processed.accel.x;
    float ay = imu.processed.accel.y;
    float az = imu.processed.accel.z;

    // normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return IMU_OKAY;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // estimated gravity direction
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // error = cross product
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // integral feedback
    if(twoKi > 0.0f){
        integralFB[0] += twoKi * ex * dt;
        integralFB[1] += twoKi * ey * dt;
        integralFB[2] += twoKi * ez * dt;

        gx += integralFB[0];
        gy += integralFB[1];
        gz += integralFB[2];
    }

    // proportional feedback
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // integrate quaternion (same as before)
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    // normalize quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    imu.processed.quat.w = q0;
    imu.processed.quat.x = q1;
    imu.processed.quat.y = q2;
    imu.processed.quat.z = q3;

    return IMU_OKAY;
}

#define GYRO_SCALING_RAD (2000.0f * 3.14159265f / (180.0f * 32767.0f))  // scale LSM6DSO raw gyro to rad
#define ACCEL_SCALING_G (16.0f / 32767.0f)  // scale LSM6DSO raw accel to g

static IMU_RETURN_TYPE imu_convert_data(const uint8_t* rx_data){
    imu.raw.rate_raw.wx = (int16_t)((rx_data[1] << 8) | rx_data[0]);
    imu.raw.rate_raw.wy = (int16_t)((rx_data[3] << 8) | rx_data[2]);
    imu.raw.rate_raw.wz = (int16_t)((rx_data[5] << 8) | rx_data[4]);


    imu.raw.accel_raw.x = (int16_t)((rx_data[7] << 8) | rx_data[6]);
    imu.raw.accel_raw.y = (int16_t)((rx_data[9] << 8) | rx_data[8]);
    imu.raw.accel_raw.z = (int16_t)((rx_data[11] << 8) | rx_data[10]);

    imu.processed.rate.wx = imu.raw.rate_raw.wx * GYRO_SCALING_RAD;
    imu.processed.rate.wy = imu.raw.rate_raw.wy * GYRO_SCALING_RAD;
    imu.processed.rate.wz = imu.raw.rate_raw.wz * GYRO_SCALING_RAD;

    imu.processed.accel.x = imu.raw.accel_raw.x * ACCEL_SCALING_G;
    imu.processed.accel.y = imu.raw.accel_raw.y * ACCEL_SCALING_G;
    imu.processed.accel.z = imu.raw.accel_raw.z * ACCEL_SCALING_G;

    return IMU_OKAY;
}

uint32_t IMU_READ_DATA(const task_info_t* task){
    #if LSM6DSO_INTERRUPT
    if (HAL_DMA_GetState(rx_dma) != HAL_DMA_STATE_READY) return 0;
    imu_convert_data(&imu_dma_rx[1]);
    imu_update_quat();
    SCHEDULER_DISABLE_TASK_BY_INDEX(imu_convert_task_index);
    #else
    uint8_t g_a_tx[13] = {0};
    uint8_t g_a_rx[13] = {0};
    g_a_tx[0] = LSM6DSO_READ_START_REG | LSM6DSO_READ;
    if(SPI_TRANSFER_BLOCKING(spi_device, &g_a_tx[0], &g_a_rx[0], 13, 1) == SPI_OKAY){
        imu_convert_data(&g_a_rx[1]);
        imu_update_quat();
    }
    #endif
    return 0;
}

#if LSM6DSO_INTERRUPT
void IMU_DATA_READY_INTERRUPT_HANDLER(void){
    if(!imu_set_up) return;
    if (HAL_DMA_GetState(rx_dma) != HAL_DMA_STATE_READY) return;
    SPI_START_CS(SPI_DEVICE_IMU);
    HAL_SPI_TransmitReceive_DMA(SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE_IMU), imu_dma_tx, imu_dma_rx, 13);
}

void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void){
    SPI_STOP_CS(SPI_DEVICE_IMU);
    SCHEDULER_ENABLE_TASK_BY_INDEX(imu_convert_task_index);
}
#else
void IMU_DATA_READY_INTERRUPT_HANDLER(void){
    Error_Handler();
}

void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void){
    Error_Handler();
}
#endif

IMU_PROCESSED_T IMU_GET_DATA(){
    return imu.processed;
}

void IMU_SET_CONFIG(imu_config_t new_config){
    config = new_config;
}

imu_config_t IMU_GET_DEFAULT_CONFIG(){
    imu_config_t temp = {0};
    temp.orientation = 0;
    temp.odr = IMU_ODR_3333Hz;
    return temp;
}


static uint8_t read_register(uint8_t address){
    uint8_t tx_buff[2] = {LSM6DSO_READ | address, 0};
    uint8_t rx_buff[2] = {0};
    SPI_TRANSFER_BLOCKING(spi_device, &tx_buff[0], &rx_buff[0], 2, 1);
    return rx_buff[1];
}

static IMU_RETURN_TYPE write_register(uint8_t address, uint8_t data){
    uint8_t tx_buff[2] = {LSM6DSO_WRITE & address, data};
    uint8_t rx_buff[2] = {0};
    SPI_TRANSFER_BLOCKING(spi_device, &tx_buff[0], &rx_buff[0], 2, 1);
    return IMU_OKAY;
}