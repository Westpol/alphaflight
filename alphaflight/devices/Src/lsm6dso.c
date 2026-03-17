#include "lsm6dso.h"
#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_it.h"
#include "timer.h"
#include <stdint.h>

#define imu_execution_delta_offset 20       // offset in us
#define imu_execution_averaging_bias 9

static IMU_T imu = {0};

static SPI_DEVICE spi_device;

#if LSM6DSO_POLLING
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
#endif

static IMU_RETURN_TYPE imu_convert_data();
static uint8_t read_register(uint8_t address);
static IMU_RETURN_TYPE write_register(uint8_t address, uint8_t data);
static IMU_RETURN_TYPE imu_setup();

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

    #if LSM6DSO_POLLING
        LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)&SPI1->RXDR);  // point DMA to SPI device
        LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&SPI1->TXDR);
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);      // enable RX finished interrupt

        imu_dma_tx[0] = LSM6DSO_READ | 0x22;    // set first byte command to read OUTX_L_G (first IMU data register)
        LL_SPI_EnableDMAReq_RX(SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE_IMU));
        LL_SPI_EnableDMAReq_TX(SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE_IMU));

        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)imu_dma_rx);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)imu_dma_tx);

        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 13);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, 13);

        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
        NVIC_EnableIRQ(EXTI2_IRQn);
    #endif

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
    }
    return 0;
}

#if LSM6DSO_POLLING
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

        LL_DMA_ClearFlag_TC0(DMA2);     // clear all unterrupt flags before starting transfer
        LL_DMA_ClearFlag_HT0(DMA2);
        LL_DMA_ClearFlag_TE0(DMA2);

        LL_DMA_ClearFlag_TC1(DMA2);
        LL_DMA_ClearFlag_HT1(DMA2);
        LL_DMA_ClearFlag_TE1(DMA2);

        SPI_START_CS(SPI_DEVICE_IMU);   // pull cs low, started before DMA config to give the IMU some time

        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 13);    // config and start DMA transfer, set state to DMA running
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, 13);

        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
        imu_dma_metadata.imu_dma_state_flags = IMU_DMA_RUNNING;
    }

    void IMU_DMA_FINISHED_INTERRUPT_HANDLER(void){
        SPI_STOP_CS(SPI_DEVICE_IMU);    // disable CS
        imu_dma_metadata.timestamp_dma_finished = MICROS32();   // timestamp package
        imu_dma_metadata.imu_dma_state_flags = IMU_DMA_READY;   // set state to DMA finished
    }
#endif

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