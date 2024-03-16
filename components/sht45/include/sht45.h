/*
 * @Author: calin.acr 
 * @Date: 2024-03-14 18:22:31 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-15 19:24:48
 */

#include "esp_err.h"
#include "driver/i2c_master.h"

#define SHT45_COMMAND_SOFT_RESET    0x94
#define SHT45_COMMAND_READ_SER_NUM  0x89

typedef enum SHT45_Mode {
    SHT45_MODE_HIGH_PRECISION_NO_HEATER = 0xFD,
    SHT45_MODE_MED_PRECISION_NO_HEATER = 0xF6,
    SHT45_MODE_LOW_PRECISION_NO_HEATER = 0xE0,
    SHT45_MODE_HIGH_PRECISION_HEATER_200MW_1S = 0x39,
    SHT45_MODE_HIGH_PRECISION_HEATER_200MW_01S = 0x32,
    SHT45_MODE_HIGH_PRECISION_HEATER_100MW_1S = 0x2F,
    SHT45_MODE_HIGH_PRECISION_HEATER_100MW_01S = 0x24,
    SHT45_MODE_HIGH_PRECISION_HEATER_20MW_1S = 0x1E,
    SHT45_MODE_HIGH_PRECISION_HEATER_20MW_01S = 0x15,
} SHT45_Mode;

extern i2c_master_dev_handle_t sht_device_handle;

void SHT45_Init(i2c_master_bus_handle_t *mbh);

void SHT45_WriteRead(uint8_t cmd, uint16_t delay, uint8_t *data, uint16_t len);
uint8_t SHT45_CRC8( uint8_t *data, uint16_t len);
void SHT45_Read(SHT45_Mode mode, uint16_t *temp_raw, float *temp_conv, uint16_t *hum_raw, float *hum_conv);
void SHT45_GetSerialNumber();
void SHT45_SoftReset();
