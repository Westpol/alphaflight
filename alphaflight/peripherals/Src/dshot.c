#include "dshot.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"

static __attribute__((section(".dma_tx"))) uint16_t dma_timings[18] = {0};
volatile static uint16_t throttle_data_buffer;

volatile static bool transmitting = false;
volatile static bool new_data = false;

static dshot_config_t config = {0};

DSHOT_RETURN_TYPE dshot_start_transmission(void);
DSHOT_RETURN_TYPE dshot_set_packet(void);

DSHOT_RETURN_TYPE DSHOT_INIT(void){
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

    DSHOT_SET_THROTTLE(0, false);

    return DSHOT_OKAY;
}

uint32_t DSHOT_TRANSMIT(const task_info_t* task){
    dshot_start_transmission();
    return 0;
}

DSHOT_RETURN_TYPE DSHOT_SET_THROTTLE(uint16_t throttle, bool armed){    // between 0 and 2000
    if(throttle > 2000) return DSHOT_FAIL;  // discard change if bad data comes in

    if(armed){
        #if DSHOT_ENABLE_IDLE_SPINNING
        throttle = UTILS_MIN_I(config.motor_max, UTILS_MAX_I(throttle, config.motor_idle)) + 47;   // Map throttle 1–2000 to ESC range 48–2047; 0 means idle spinning
        #else
        if(throttle != 0) throttle = UTILS_MIN_UI(config.motor_max, UTILS_MAX_UI(throttle, config.motor_idle)) + 47;   // Map throttle 1–2000 to ESC range 48–2047; 0 remains disarmed
        #endif
    }
    else{
        throttle = 0;   // throttle set to disarmed
    }

    if(!transmitting){
        throttle_data_buffer = throttle;
        dshot_set_packet();
        return DSHOT_OKAY;
    }
    new_data = true;
    throttle_data_buffer = throttle;
    return DSHOT_OKAY;
}

DSHOT_RETURN_TYPE dshot_start_transmission(void){
    if(transmitting) return DSHOT_FAIL;

    if(new_data){
        dshot_set_packet();    // prepare DMA buffer in case new data came in while transferring DMA
        new_data = false;
    }

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)dma_timings);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)&TIM1->CCR1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 18);

    LL_TIM_EnableDMAReq_UPDATE(TIM1);

    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

    LL_TIM_EnableAllOutputs(TIM1);

    transmitting = true;
    
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

    LL_TIM_GenerateEvent_UPDATE(TIM1);  // load first DSHOT bit

    LL_TIM_EnableCounter(TIM1);


    return DSHOT_OKAY;
}

#define DSHOT_TELEMETRY false
#define DSHOT600_CCR_HIGH 150
#define DSHOT600_CCR_LOW 75
DSHOT_RETURN_TYPE dshot_set_packet(void){
    uint16_t packet;

    packet = throttle_data_buffer << 1 | (DSHOT_TELEMETRY ? 1 : 0);  // "generate" DSHOT packet data

    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;  // get raw CRC from DSHOT packet data

    packet = (packet << 4) | csum;  // copy CRC into packet

    for (uint8_t i = 0; i < 16; i++) {  // turn packet bits into TIM CCR array
    dma_timings[i] = (packet >> (15 - i)) & 0x01 ? DSHOT600_CCR_HIGH : DSHOT600_CCR_LOW;
    }

    dma_timings[16] = 0;
    dma_timings[17] = 0;
    return DSHOT_OKAY;
}

DSHOT_RETURN_TYPE DSHOT_DMA_CPLT_CALLBACK(void){
    //__IO uint32_t flags = DMA1->LISR;
    DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;  // clear all flags
    // add error flag handling later
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableDMAReq_UPDATE(TIM1);
    transmitting = false;
    return DSHOT_OKAY;
}

DSHOT_RETURN_TYPE DSHOT_SET_CONFIG(dshot_config_t conf){
    config = conf;
    return DSHOT_OKAY;
}


dshot_config_t DSHOT_GET_CONFIG(void){
    return config;
}


dshot_config_t DSHOT_GET_DEFAULT_CONFIG(void){
    dshot_config_t temp = {0};
    temp.motor_idle = 50;
    temp.motor_max = 2000;
    return temp;
}