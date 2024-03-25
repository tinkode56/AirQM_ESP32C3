/*
 * @Author: calin.acr 
 * @Date: 2024-03-19 16:22:34 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-25 17:29:16
 */

#if !defined(_SENSEAIR_S8_H)
#define _SENSEAIR_S8_H

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "mbcontroller.h"

/**
 * MODBUS message
 * 
 * DevADR | func code | start address | quantity of regs | CRCL CRCH
 * -----------------------------------------------------------------
 * 0x68   |  0x04     | 0x00 0x03     | 0x00 0x01        | 0xC8 0xF3
*/

#define SENSEAIR_UART_PORT UART_NUM_0
#define SENSEAIR_RX_PIN GPIO_NUM_20
#define SENSEAIR_TX_PIN GPIO_NUM_21

#define CHECK_CO2_BGCAL(x)      ((x) & 0x20) ? 1 : 0
#define CHECK_CO2_N_BGCAL(x)    ((x) & 0x40) ? 1 : 0

#define CHECK_ERROR_ANY(x)      ((x) & 0x7F) ? 1 : 0
#define CHECK_ERROR_FATAL(x)    ((x) & 0x01) ? 1 : 0
#define CHECK_ERROR_OFFSET(x)   ((x) & 0x02) ? 1 : 0
#define CHECK_ERROR_ALGO(x)     ((x) & 0x04) ? 1 : 0
#define CHECK_ERROR_OUTPUT(x)   ((x) & 0x08) ? 1 : 0
#define CHECK_ERROR_DIAG(x)     ((x) & 0x10) ? 1 : 0
#define CHECK_ERROR_OOR(x)      ((x) & 0x20) ? 1 : 0
#define CHECK_ERROR_MEMORY(x)   ((x) & 0x40) ? 1 : 0

enum MBAddr {
    MB_DEVICE_S8 = 0x68,
};

enum IRAddr {
    METER_STATUS = 0,
    ALARM_STATUS,
    OUTPUT_STATUS,
    SPACE_CO2,
    PWM_OUTPUT = 21,
    S_TYPE_ID_H = 25,
    S_TYPE_ID_L,
    MEMMAP_VER,
    FW_VER,
    S_ID_H,
    S_ID_L,
};

enum HRAddr {
    ACK_REG = 0,
    SPECIAL_CMD,
    ABC_PERIOD = 31,
};

enum Commands {
    S8_CMD_RHR = 0x03,
    S8_CMD_RIR = 0x04,
    S8_CMD_WSR = 0x06,
};

enum CalibrationType {
    CO2_BG_CAL = 0x06,
    CO2_N_BG_CAL = 0x07,
};

extern mb_param_request_t s8_request;

void SenseairS8_Init(void);
void S8_ReadCO2(uint16_t *co2);
void S8_ReadStatus(uint16_t *status);
void S8_ReadCO2andStatus(uint16_t *co2, uint16_t *status);
uint8_t S8_BgCalibration(uint8_t cal_type);
void S8_ReadABC(uint16_t *abc_value);
void S8_SetABC(uint16_t *abc_value);
void S8_ReadSensorTypeID(uint32_t *value);
void S8_ReadMemMapVers(uint16_t *value);
void S8_ReadFWVers(uint16_t *value);
void S8_ReadSensorID(uint32_t *value);

#endif // _SENSEAIR_S8_H
