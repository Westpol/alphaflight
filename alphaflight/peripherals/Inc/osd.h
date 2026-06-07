#include "common.h"
#include "stm32h7xx_hal.h"


#ifndef OSD_H
#define OSD_H

#define OSD_DMA_BUFFER_SIZE (8 * STM32_WORD_SIZE)

typedef enum{
    OSD_OKAY,
    OSD_FAIL
} OSD_RETURN_TYPE;

OSD_RETURN_TYPE OSD_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* dma);



/******************************************************************
 *
 * MultiWii Serial Protocol - 
 * 
 ******************************************************************/

// Multiwii Serial Protocol 0 
#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1  // increment when major changes are made
#define API_VERSION_MINOR                   44 // increment after a release

/******************************************************************
 *
 * MSP Messages - 
 * 
 * To Multiwii developers/committers : do not add new MSP messages
 * without a proper argumentation/agreement on the forum.
 * 
 * Range id [50-99] won't be assigned and can therefore be used for
 * any custom multiwii fork without further MSP id conflict
 * 
 ******************************************************************/

#define MSP_API_VERSION          1    //out message
#define MSP_FC_VARIANT           2    //out message
#define MSP_FC_VERSION           3    //out message
#define MSP_BOARD_INFO           4    //out message
#define MSP_BUILD_INFO           5    //out message

#define MSP_NAME                 10   //out message          Returns user set board name - betaflight
#define MSP_SET_NAME             11   //in message           Sets board name - betaflight

/******************************************************************
 *
 *  REEFWING SPECIFIC MSG IDS
 * 
 ******************************************************************/

#define MSP_IMU_ODR              50   //out message           Returns the IMU sample rates for gyro, acc and mag
#define MSP_IMU_BIAS             51   //out message           Returns the x,y and z bias offsets for gyro, acc and mag

#define MSP_IMU_CALIBRATION      75   //in message            no param
#define MSP_SET_ARM              76   //in message            no param - forces drone into ARMED state
#define MSP_SET_DISARM           77   //in message            no param - forces drone into DISARMED state
#define MSP_STOP_STREAM          78   //in message            no param - stops all data streaming (PID, SBUS & IMU)
#define MSP_CONFIG_CLOSE         79   //in message            no param - Configurator about to disconnect

/******************************************************************
 *
 *  CLASSIC MSG IDS
 * 
 ******************************************************************/

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

/******************************************************************
 *
 * MSP Configuration Defines - 
 * 
 ******************************************************************/

//  unique flight controller IDENT - in accordance with Betaflight MSP guidelines
#define REEFWING_IDENTIFIER "RWNG"
#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
#define BETAFLIGHT_IDENTIFIER "BTFL"
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"

// flags for msp_status_ex_t.sensor and msp_status_t.sensor
#define MSP_STATUS_SENSOR_ACC    1
#define MSP_STATUS_SENSOR_BARO   2
#define MSP_STATUS_SENSOR_MAG    4
#define MSP_STATUS_SENSOR_GPS    8
#define MSP_STATUS_SENSOR_SONAR 16

#define MSP_MAX_SUPPORTED_SERVOS 8
#define MSP_MAX_SERVO_RULES (2 * MSP_MAX_SUPPORTED_SERVOS)
#define MSP_MAX_SUPPORTED_MOTORS 8
#define MSP_MAX_SUPPORTED_CHANNELS 16
#define MSP_QUAD_MOTORS 4
#define MSP_PAYLOAD_SIZE  4

// values for msp_raw_gps_t.fixType
#define MSP_GPS_NO_FIX 0
#define MSP_GPS_FIX_2D 1
#define MSP_GPS_FIX_3D 2

// values for msp_nav_status_t.mode
#define MSP_NAV_STATUS_MODE_NONE   0
#define MSP_NAV_STATUS_MODE_HOLD   1
#define MSP_NAV_STATUS_MODE_RTH    2
#define MSP_NAV_STATUS_MODE_NAV    3
#define MSP_NAV_STATUS_MODE_EMERG 15

// values for msp_nav_status_t.state
#define MSP_NAV_STATUS_STATE_NONE                0  // None
#define MSP_NAV_STATUS_STATE_RTH_START           1  // RTH Start
#define MSP_NAV_STATUS_STATE_RTH_ENROUTE         2  // RTH Enroute
#define MSP_NAV_STATUS_STATE_HOLD_INFINIT        3  // PosHold infinit
#define MSP_NAV_STATUS_STATE_HOLD_TIMED          4  // PosHold timed
#define MSP_NAV_STATUS_STATE_WP_ENROUTE          5  // WP Enroute
#define MSP_NAV_STATUS_STATE_PROCESS_NEXT        6  // Process next
#define MSP_NAV_STATUS_STATE_DO_JUMP             7  // Jump
#define MSP_NAV_STATUS_STATE_LAND_START          8  // Start Land
#define MSP_NAV_STATUS_STATE_LAND_IN_PROGRESS    9  // Land in Progress
#define MSP_NAV_STATUS_STATE_LANDED             10  // Landed
#define MSP_NAV_STATUS_STATE_LAND_SETTLE        11  // Settling before land
#define MSP_NAV_STATUS_STATE_LAND_START_DESCENT 12  // Start descent

// values for msp_nav_status_t.activeWpAction, msp_set_wp_t.action
#define MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT 0x01
#define MSP_NAV_STATUS_WAYPOINT_ACTION_RTH      0x04

// values for msp_nav_status_t.error
#define MSP_NAV_STATUS_ERROR_NONE               0   // All systems clear
#define MSP_NAV_STATUS_ERROR_TOOFAR             1   // Next waypoint distance is more than safety distance
#define MSP_NAV_STATUS_ERROR_SPOILED_GPS        2   // GPS reception is compromised - Nav paused - copter is adrift !
#define MSP_NAV_STATUS_ERROR_WP_CRC             3   // CRC error reading WP data from EEPROM - Nav stopped
#define MSP_NAV_STATUS_ERROR_FINISH             4   // End flag detected, navigation finished
#define MSP_NAV_STATUS_ERROR_TIMEWAIT           5   // Waiting for poshold timer
#define MSP_NAV_STATUS_ERROR_INVALID_JUMP       6   // Invalid jump target detected, aborting
#define MSP_NAV_STATUS_ERROR_INVALID_DATA       7   // Invalid mission step action code, aborting, copter is adrift
#define MSP_NAV_STATUS_ERROR_WAIT_FOR_RTH_ALT   8   // Waiting to reach RTH Altitude
#define MSP_NAV_STATUS_ERROR_GPS_FIX_LOST       9   // Gps fix lost, aborting mission
#define MSP_NAV_STATUS_ERROR_DISARMED          10   // NAV engine disabled due disarm
#define MSP_NAV_STATUS_ERROR_LANDING           11   // Landing

// MSP_FEATURE mask
#define MSP_FEATURE_RX_PPM              (1 <<  0)
#define MSP_FEATURE_VBAT                (1 <<  1)
#define MSP_FEATURE_UNUSED_1            (1 <<  2)
#define MSP_FEATURE_RX_SERIAL           (1 <<  3)
#define MSP_FEATURE_MOTOR_STOP          (1 <<  4)
#define MSP_FEATURE_SERVO_TILT          (1 <<  5)
#define MSP_FEATURE_SOFTSERIAL          (1 <<  6)
#define MSP_FEATURE_GPS                 (1 <<  7)
#define MSP_FEATURE_UNUSED_3            (1 <<  8)         // was FEATURE_FAILSAFE
#define MSP_FEATURE_UNUSED_4            (1 <<  9)         // was FEATURE_SONAR
#define MSP_FEATURE_TELEMETRY           (1 << 10)
#define MSP_FEATURE_CURRENT_METER       (1 << 11)
#define MSP_FEATURE_3D                  (1 << 12)
#define MSP_FEATURE_RX_PARALLEL_PWM     (1 << 13)
#define MSP_FEATURE_RX_MSP              (1 << 14)
#define MSP_FEATURE_RSSI_ADC            (1 << 15)
#define MSP_FEATURE_LED_STRIP           (1 << 16)
#define MSP_FEATURE_DASHBOARD           (1 << 17)
#define MSP_FEATURE_UNUSED_2            (1 << 18)
#define MSP_FEATURE_BLACKBOX            (1 << 19)
#define MSP_FEATURE_CHANNEL_FORWARDING  (1 << 20)
#define MSP_FEATURE_TRANSPONDER         (1 << 21)
#define MSP_FEATURE_AIRMODE             (1 << 22)
#define MSP_FEATURE_SUPEREXPO_RATES     (1 << 23)
#define MSP_FEATURE_VTX                 (1 << 24)
#define MSP_FEATURE_RX_SPI              (1 << 25)
#define MSP_FEATURE_SOFTSPI             (1 << 26)
#define MSP_FEATURE_PWM_SERVO_DRIVER    (1 << 27)
#define MSP_FEATURE_PWM_OUTPUT_ENABLE   (1 << 28)
#define MSP_FEATURE_OSD                 (1 << 29)

// values for msp_current_meter_config_t.currentMeterType
#define MSP_CURRENT_SENSOR_NONE    0
#define MSP_CURRENT_SENSOR_ADC     1
#define MSP_CURRENT_SENSOR_VIRTUAL 2
#define MSP_CURRENT_SENSOR_MAX     CURRENT_SENSOR_VIRTUAL

// msp_rx_config_t.serialrx_provider
#define MSP_SERIALRX_SPEKTRUM1024      0
#define MSP_SERIALRX_SPEKTRUM2048      1
#define MSP_SERIALRX_SBUS              2
#define MSP_SERIALRX_SUMD              3
#define MSP_SERIALRX_SUMH              4
#define MSP_SERIALRX_XBUS_MODE_B       5
#define MSP_SERIALRX_XBUS_MODE_B_RJ01  6
#define MSP_SERIALRX_IBUS              7
#define MSP_SERIALRX_JETIEXBUS         8
#define MSP_SERIALRX_CRSF              9


// msp_rx_config_t.rx_spi_protocol values
#define MSP_SPI_PROT_NRF24RX_V202_250K 0
#define MSP_SPI_PROT_NRF24RX_V202_1M   1
#define MSP_SPI_PROT_NRF24RX_SYMA_X    2
#define MSP_SPI_PROT_NRF24RX_SYMA_X5C  3
#define MSP_SPI_PROT_NRF24RX_CX10      4
#define MSP_SPI_PROT_NRF24RX_CX10A     5
#define MSP_SPI_PROT_NRF24RX_H8_3D     6
#define MSP_SPI_PROT_NRF24RX_INAV      7

#define MSP_MAX_MAPPABLE_RX_INPUTS 8

// values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
#define MSP_SENSOR_ALIGN_CW0_DEG        1
#define MSP_SENSOR_ALIGN_CW90_DEG       2
#define MSP_SENSOR_ALIGN_CW180_DEG      3
#define MSP_SENSOR_ALIGN_CW270_DEG      4
#define MSP_SENSOR_ALIGN_CW0_DEG_FLIP   5
#define MSP_SENSOR_ALIGN_CW90_DEG_FLIP  6
#define MSP_SENSOR_ALIGN_CW180_DEG_FLIP 7
#define MSP_SENSOR_ALIGN_CW270_DEG_FLIP 8

//  multitype ID's - used in MultiWiiConf
#define TRI            1 
#define QUADP          2 
#define QUADX          3 
#define BI             4 
#define GIMBAL         5 
#define Y6             6 
#define HEX6           7 
#define FLYING_WING    8 
#define Y4             9 
#define HEX6X          10 
#define OCTOX8         11 
#define OCTOFLATX      12 
#define OCTOFLATP      13 
#define AIRPLANE       14 
#define HELI_120_CCPM  15 
#define HELI_90_DEG    16 
#define VTAIL4         17 
#define HEX6H          18 
#define PPM_TO_SERVO   19 
#define DUALCOPTER     20 
#define SINGLECOPTER   21 

#endif