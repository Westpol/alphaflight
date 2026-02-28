#include "lsm6dso.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_spi.h"
#include "timer.h"

#define imu_execution_delta_offset 20       // offset in us
#define imu_execution_averaging_bias 9

#define IMU_DMA_PROCESSED 0x00
#define IMU_DMA_READY 0x01
#define IMU_DMA_RUNNING 0x02

__attribute__((section(".dma_rx"))) static uint8_t imu_dma_rx[STM32_WORD_SIZE] = {0};
__attribute__((section(".dma_tx"))) static uint8_t imu_dma_tx[STM32_WORD_SIZE] = {0};

volatile static struct{
    uint32_t timestamp_dma_finished;
    uint32_t timestamp_dma_finished_last;
    uint32_t execution_delta_average;
    uint8_t imu_dma_state_flags;        // three states: 0x00 -> data processed, 0x01 -> new data (DMA finished), 0x02 -> DMA running
    uint16_t phase_delay;
    uint32_t num_missed_measurements;
} imu_dma_metadata;

IMU_RETURN_TYPE IMU_INIT(){
    uint8_t tx_buff_imu[2] = {0x8F, 0x00};
    uint8_t rx_buff_imu[2] = {0x00, 0x00};
    SPI_TRANSFER_FIFO(SPI_DEVICE_IMU, tx_buff_imu, rx_buff_imu, 2);
    if(rx_buff_imu[1] != 108) return IMU_WRONG_ID;
    imu_dma_tx[0] = 0x80 | 0x22;    // set first byte command to read OUTX_L_G (first IMU data register)
    return IMU_OKAY;
}

uint32_t IMU_CONVERT_DATA(const task_info_t *task){    // converting register values in read gyro and accel data, returns execution delta for next predicted DRDY + dma transfer execution time
    if(imu_dma_metadata.imu_dma_state_flags != IMU_DMA_READY) return 0;
    imu_dma_metadata.phase_delay = MICROS32() - imu_dma_metadata.timestamp_dma_finished;
    // TODO: parse data
    imu_dma_metadata.imu_dma_state_flags = IMU_DMA_PROCESSED;

    uint32_t drdy_delta_time = imu_dma_metadata.timestamp_dma_finished - imu_dma_metadata.timestamp_dma_finished_last;
    imu_dma_metadata.timestamp_dma_finished_last = imu_dma_metadata.timestamp_dma_finished;

    imu_dma_metadata.execution_delta_average = (imu_dma_metadata.execution_delta_average * imu_execution_averaging_bias + drdy_delta_time) / (imu_execution_averaging_bias + 1);
    return imu_dma_metadata.execution_delta_average;
}

void IMU_DATA_READY_INTERRUPT_HANDLER(void){
    if(imu_dma_metadata.imu_dma_state_flags == IMU_DMA_RUNNING) return;        // dma still running, skip everything
    if(imu_dma_metadata.imu_dma_state_flags == IMU_DMA_READY) imu_dma_metadata.num_missed_measurements++;       // last packet hasn't yet been parsed, save the skip in variable and start DMA

    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)imu_dma_rx);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)imu_dma_tx);

    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 13);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, 13);

    LL_SPI_EnableDMAReq_RX(SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE_IMU));
    LL_SPI_EnableDMAReq_TX(SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE_IMU));

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
    imu_dma_metadata.imu_dma_state_flags = IMU_DMA_RUNNING;
}

void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void){
    imu_dma_metadata.timestamp_dma_finished = MICROS32();
    imu_dma_metadata.imu_dma_state_flags = IMU_DMA_READY;
}