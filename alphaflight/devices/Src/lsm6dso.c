#include "lsm6dso.h"
#include "main.h"
#include "timer.h"

#define imu_execution_delta_offset 20       // offset in us
#define imu_execution_averaging_bias 9

static IMU_T imu = {0};

static SPI_DEVICE spi_device;

static imu_config_t config = {0};

static uint32_t last_integration = 0;


static IMU_RETURN_TYPE imu_convert_data();
static uint8_t read_register(uint8_t address);
static IMU_RETURN_TYPE write_register(uint8_t address, uint8_t data);
static IMU_RETURN_TYPE imu_setup();
static IMU_RETURN_TYPE imu_update_quat(void);


static IMU_RETURN_TYPE imu_setup(){
    write_register(LSM6DSO_CRTL3_C_ADDRESS, LSM6DSO_CTRL3_C_RESET);
    while((read_register(LSM6DSO_CRTL3_C_ADDRESS) & 0x01));
    write_register(LSM6DSO_CTRL9_XL_ADDRESS, LSM6DSO_CTRL9_XL_I3C_DISABLE);
    write_register(LSM6DSO_CTRL1_XL_ADDRESS, (LSM6DSO_CTRL1_XL_ODR_1666 | LSM6DSO_CTRL1_XL_FS_16) & LSM6DSO_CTRL1_XL_MASK_AND);
    write_register(LSM6DSO_CTRL2_G_ADDRESS, (LSM6DSO_CTRL2_G_ODR_1666 | LSM6DSO_CTRL2_G_FS_2500) & LSM6DSO_CTRL2_G_MASK_AND);
    write_register(LSM6DSO_CRTL4_C_ADDRESS, LSM6DSO_CTRL4_C_DRDY_MASK & LSM6DSO_CTRL4_C_MASK_AND);
    #if LSM6DSO_POLLING
        write_register(LSM6DSO_INT1_CTRL_ADDRESS, LSM6DSO_INT1_CTRL_DRDY_G);
    #endif
    return IMU_OKAY;
}


IMU_RETURN_TYPE IMU_INIT(SPI_DEVICE device){
    spi_device = device;
    imu.processed.quat.w = 1.0f;    // set to standard orientation
    if(read_register(0x8F) != 108) return IMU_WRONG_ID; // check if IMU is registered

    if(imu_setup() != IMU_OKAY) return IMU_SETUP_FAILED;

    return IMU_OKAY;
}

static IMU_RETURN_TYPE imu_update_quat(void){
    float dt = (float)(MICROS32() - last_integration) / 1000000.0f;
    last_integration = MICROS32();

    float q[4] = {
        imu.processed.quat.w,
        imu.processed.quat.x,
        imu.processed.quat.y,
        imu.processed.quat.z
    };

    float omega[4] = {
        0.0f,
        imu.processed.rate.wx,
        imu.processed.rate.wy,
        imu.processed.rate.wz
    };

    float dq[4];
    UTILS_QUATERNION_PRODUCT(q, omega, dq);

    // q_dot = 0.5 * dq
    for (int i = 0; i < 4; i++){
        q[i] += 0.5f * dq[i] * dt;
    }

    // normalize
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++){
        q[i] /= norm;
    }

    imu.processed.quat.w = q[0];
    imu.processed.quat.x = q[1];
    imu.processed.quat.y = q[2];
    imu.processed.quat.z = q[3];

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
    uint8_t g_a_tx[13] = {0};
    uint8_t g_a_rx[13] = {0};
    g_a_tx[0] = LSM6DSO_READ_START_REG | LSM6DSO_READ;
    if(SPI_TRANSFER_BLOCKING(spi_device, &g_a_tx[0], &g_a_rx[0], 13, 1) == SPI_OKAY){
        imu_convert_data(&g_a_rx[1]);
        imu_update_quat();
    }
    return 0;
}

#if !LSM6DSO_POLLING
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