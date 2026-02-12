#include "lsm6dso.h"

IMU_RETURN_TYPE IMU_INIT(){
    uint8_t tx_buff_imu[2] = {0x8F, 0x00};
    uint8_t rx_buff_imu[2] = {0x00, 0x00};
    SPI_TRANSFER_FIFO(SPI_DEVICE_IMU, tx_buff_imu, rx_buff_imu, 2);
    if(rx_buff_imu[1] != 108) return IMU_WRONG_ID;
    return IMU_OKAY;
}