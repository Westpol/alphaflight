#include "bmp390.h"

#include <math.h>

#include "timer.h"
#include "usb.h"

static BARO_T baro = {0};

static SPI_DEVICE spi_device;

static Baro_Calibration baro_calibration = {0};

static uint8_t read_register(uint8_t address);
static BARO_RETURN_TYPE write_register(uint8_t address, uint8_t data);
static BARO_RETURN_TYPE baro_setup();
static BARO_RETURN_TYPE baro_get_calib_values();
static BARO_RETURN_TYPE baro_get_base();
static BARO_RETURN_TYPE baro_convert_data_no_height(uint8_t* rx_val);
static BARO_RETURN_TYPE baro_convert_data(uint8_t* rx_val);
static float baro_get_altitude(float p, float p0);
static float baro_compensate_temperature(uint32_t uncomp_temp, Baro_Calibration *calib_data);
static float baro_compensate_pressure(uint32_t uncomp_press, Baro_Calibration *calib_data);

uint32_t BARO_READ_DATA(const task_info_t* task){
    uint8_t baro_read_tx[8] = {0};
    uint8_t baro_read_rx[8] = {0};
    baro_read_tx[0] = BMP390_READ | BMP390_DATA_START_ADDRESS;
    SPI_TRANSFER_BLOCKING(spi_device, baro_read_tx, baro_read_rx, 8, 1);

    baro_convert_data(&baro_read_rx[2]);

    return 0;
}

BARO_RETURN_TYPE BARO_INIT(SPI_DEVICE device){
    spi_device = device;
    if(read_register(BMP390_ID_ADDRESS) != BMP390_ID_VALUE) return BARO_WRONG_ID;

    BARO_RETURN_TYPE setup_ret = baro_setup();
    if(setup_ret != BARO_OKAY) return setup_ret;

    uint32_t now = MICROS32();
    while((MICROS32() - now) <= 1000000);    // wait 1s until data has sabilized

    baro_get_base();

    return BARO_OKAY;
}

BARO_PROCESSED_T BARO_GET_DATA(){
    return baro.processed;
}

BARO_T BARO_GET_DATA_RAW(){
	return baro;
}

#define VERTICAL_SPEED_ALPHA 0.5f
static BARO_RETURN_TYPE baro_convert_data(uint8_t* rx_val){
    baro.raw.pressure = ((uint32_t)rx_val[2] << 16) | ((uint32_t)rx_val[1] << 8) | rx_val[0];
    baro.raw.temp = ((uint32_t)rx_val[5] << 16) | ((uint32_t)rx_val[4] << 8) | rx_val[3];
    baro.processed.temp = baro_compensate_temperature(baro.raw.temp, &baro_calibration);
    baro.processed.pressure = baro_compensate_pressure(baro.raw.pressure, &baro_calibration);
    baro.processed.height = baro_get_altitude(baro.processed.pressure, baro.processing.pressure_base);
	uint32_t now = MICROS32();
	uint32_t delta = now - baro.processing.pressure_last_timestamp;
	if(delta > 0){
		baro.processed.vertical_speed = baro.processed.vertical_speed * (1.0f - VERTICAL_SPEED_ALPHA) + ((baro.processed.height - baro.processing.height_last) * 100000000.0f / (float)delta) * VERTICAL_SPEED_ALPHA;
	}
	baro.processing.height_last = baro.processed.height;
	baro.processing.pressure_last_timestamp = now;
    return BARO_OKAY;
}

static float baro_get_altitude(float p, float p0){
    return 44330.0f * (1.0f - powf(p / p0, 0.1903f));
}

static BARO_RETURN_TYPE baro_get_base(){
    do{
        uint8_t baro_read_tx[8] = {0};
        uint8_t baro_read_rx[8] = {0};
        baro_read_tx[0] = BMP390_READ | BMP390_DATA_START_ADDRESS;
        SPI_TRANSFER_BLOCKING(spi_device, baro_read_tx, baro_read_rx, 8, 1);

        baro_convert_data_no_height(&baro_read_rx[2]);
    } while(baro.processed.pressure < 1000);
    baro.processing.pressure_base = baro.processed.pressure;
    return BARO_OKAY;
}

static BARO_RETURN_TYPE baro_setup(){
    write_register(BMP390_CMD_ADDRESS, BMP390_CMD_RESET);   // reset BMP390 to default
    uint32_t now = MICROS32();
    while((MICROS32() - now) <= 10000);

    write_register(BMP390_OSR_ADDRESS, BMP390_OSR_PRESS_X16 | BMP390_OSR_TEMP_X2);
    write_register(BMP390_ODR_ADDRESS, BMP390_ODR_25HZ);
    write_register(BMP390_IIR_ADDRESS, BMP390_IIR_COEF_15);
    write_register(BMP390_PWR_ADDRESS, BMP390_PWR_PRESS_ON | BMP390_PWR_TEMP_ON | BMP390_PWR_MODE_NORMAL);

    BARO_RETURN_TYPE ret = baro_get_calib_values();
    if(ret != BARO_OKAY) return ret;

    return BARO_OKAY;
}

static BARO_RETURN_TYPE baro_get_calib_values(){
    uint8_t calib_rx_buffer_temp[23] = {0};
    uint8_t calib_tx_buffer[23] = {0};
    calib_tx_buffer[0] = BMP390_READ | 0x31;

	SPI_TRANSFER_BLOCKING(spi_device, calib_tx_buffer, calib_rx_buffer_temp, 23, 10);

    uint8_t* calib_rx_buffer = &calib_rx_buffer_temp[2];

	baro_calibration.NVM_PAR_T1 = ((uint16_t)calib_rx_buffer[1] << 8) | calib_rx_buffer[0];
	baro_calibration.NVM_PAR_T2 = ((uint16_t)calib_rx_buffer[3] << 8) | calib_rx_buffer[2];
	baro_calibration.NVM_PAR_T3 = (int8_t)calib_rx_buffer[4];
	baro_calibration.NVM_PAR_P1 = (int16_t)((uint16_t)calib_rx_buffer[6] << 8) | calib_rx_buffer[5];
	baro_calibration.NVM_PAR_P2 = (int16_t)((uint16_t)calib_rx_buffer[8] << 8) | calib_rx_buffer[7];
	baro_calibration.NVM_PAR_P3 = (int8_t)calib_rx_buffer[9];
	baro_calibration.NVM_PAR_P4 = (int8_t)calib_rx_buffer[10];
	baro_calibration.NVM_PAR_P5 = ((uint16_t)calib_rx_buffer[12] << 8) | calib_rx_buffer[11];
	baro_calibration.NVM_PAR_P6 = ((uint16_t)calib_rx_buffer[14] << 8) | calib_rx_buffer[13];
	baro_calibration.NVM_PAR_P7 = (int8_t)calib_rx_buffer[15];
	baro_calibration.NVM_PAR_P8 = (int8_t)calib_rx_buffer[16];
	baro_calibration.NVM_PAR_P9 = (int16_t)((uint16_t)calib_rx_buffer[18] << 8) | calib_rx_buffer[17];
	baro_calibration.NVM_PAR_P10 = (int8_t)calib_rx_buffer[19];
	baro_calibration.NVM_PAR_P11 = (int8_t)calib_rx_buffer[20];

	baro_calibration.par_t1 = (float)baro_calibration.NVM_PAR_T1 / pow(2, -8);
	baro_calibration.par_t2 = (float)baro_calibration.NVM_PAR_T2 / pow(2, 30);
	baro_calibration.par_t3 = (float)baro_calibration.NVM_PAR_T3 / pow(2, 48);
	baro_calibration.par_p1 = ((float)baro_calibration.NVM_PAR_P1 - pow(2, 14)) / pow(2, 20);
	baro_calibration.par_p2 = ((float)baro_calibration.NVM_PAR_P2 - pow(2, 14)) / pow(2, 29);
	baro_calibration.par_p3 = (float)baro_calibration.NVM_PAR_P3 / pow(2, 32);
	baro_calibration.par_p4 = (float)baro_calibration.NVM_PAR_P4 / pow(2, 37);
	baro_calibration.par_p5 = (float)baro_calibration.NVM_PAR_P5 / pow(2, -3);
	baro_calibration.par_p6 = (float)baro_calibration.NVM_PAR_P6 / pow(2, 6);
	baro_calibration.par_p7 = (float)baro_calibration.NVM_PAR_P7 / pow(2, 8);
	baro_calibration.par_p8 = (float)baro_calibration.NVM_PAR_P8 / pow(2, 15);
	baro_calibration.par_p9 = (float)baro_calibration.NVM_PAR_P9 / pow(2, 48);
	baro_calibration.par_p10 = (float)baro_calibration.NVM_PAR_P10 / pow(2, 48);
	baro_calibration.par_p11 = (float)baro_calibration.NVM_PAR_P11 / pow(2, 65);
    return BARO_OKAY;
}

static BARO_RETURN_TYPE baro_convert_data_no_height(uint8_t* rx_val){
    baro.raw.pressure = ((uint32_t)rx_val[2] << 16) | ((uint32_t)rx_val[1] << 8) | rx_val[0];
    baro.raw.temp = ((uint32_t)rx_val[5] << 16) | ((uint32_t)rx_val[4] << 8) | rx_val[3];
    baro.processed.temp = baro_compensate_temperature(baro.raw.temp, &baro_calibration);
    baro.processed.pressure = baro_compensate_pressure(baro.raw.pressure, &baro_calibration);
    return BARO_OKAY;
}

static float baro_compensate_temperature(uint32_t uncomp_temp, Baro_Calibration *calib_data){
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
	/* Returns compensated temperature */
	return calib_data->t_lin;
}

static float baro_compensate_pressure(uint32_t uncomp_press, Baro_Calibration *calib_data){
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

static uint8_t read_register(uint8_t address){
    uint8_t tx_buff[3] = {BMP390_READ | address, 0, 0};
    uint8_t rx_buff[3] = {0};
    SPI_TRANSFER_BLOCKING(spi_device, &tx_buff[0], &rx_buff[0], 3, 1);
    return rx_buff[2];
}

static BARO_RETURN_TYPE write_register(uint8_t address, uint8_t data){
    uint8_t tx_buff[2] = {BMP390_WRITE & address, data};
    uint8_t rx_buff[2] = {0};
    if(SPI_TRANSFER_BLOCKING(spi_device, &tx_buff[0], &rx_buff[0], 2, 1) != SPI_OKAY) return BARO_FAIL;
    return BARO_OKAY;
}

BARO_RETURN_TYPE BARO_PRINT_DATA(void){
	USB_PRINTLN("Temperature: %f°C\nPressure: %fhPa\nHeight: %fm\n", baro.processed.temp, baro.processed.pressure, baro.processed.height);
	return BARO_OKAY;
}