/*
 * @Author: calin.acr 
 * @Date: 2024-03-14 18:22:35 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-15 21:57:08
 */
#include <stdio.h>
#include "esp_log.h"
#include "sht45.h"
#include "string.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

i2c_master_dev_handle_t sht_device_handle = NULL;

void SHT45_Init(i2c_master_bus_handle_t *mbh)
{
    i2c_device_config_t sht_i2c_device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x44,
        .scl_speed_hz = 400000,
    };
    // Register SHT45 sensor to the I2C master bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*mbh, &sht_i2c_device, &sht_device_handle));
}

uint8_t SHT45_CRC8( uint8_t *data, uint16_t len)
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;
    uint16_t i, j;

    for (j = len; j != 0; --j)
    {
        crc ^= *data++;
        for (i = 8; i != 0; --i)
        {
            crc = (crc & 0x80) ? (crc << 1 ) ^ POLYNOMIAL : (crc << 1);
        }
    }
    
    return crc;
}

void SHT45_WriteRead(uint8_t cmd, uint16_t delay, uint8_t *data, uint16_t len)
{
    ESP_ERROR_CHECK(i2c_master_transmit(sht_device_handle, &cmd, 1, -1));

    vTaskDelay(pdMS_TO_TICKS(delay));

    if (len != 0)
    {
        ESP_ERROR_CHECK(i2c_master_receive(sht_device_handle, data, len, -1));
    }
}

void SHT45_SoftReset()
{
    SHT45_WriteRead(SHT45_COMMAND_SOFT_RESET, 10, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void SHT45_Read(SHT45_Mode mode, uint16_t *temp_raw, float *temp_conv, uint16_t *hum_raw, float *hum_conv)
{
    uint8_t buffer[6];

    switch (mode)
    {
    case SHT45_MODE_HIGH_PRECISION_NO_HEATER:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_NO_HEATER, 20, buffer, 6);
        break;
    case SHT45_MODE_MED_PRECISION_NO_HEATER:
        SHT45_WriteRead(SHT45_MODE_MED_PRECISION_NO_HEATER, 5, buffer, 6);
        break;
    case SHT45_MODE_LOW_PRECISION_NO_HEATER:
        SHT45_WriteRead(SHT45_MODE_LOW_PRECISION_NO_HEATER, 2, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_200MW_1S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_200MW_1S, 1100, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_200MW_01S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_200MW_01S, 110, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_100MW_1S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_100MW_1S, 1100, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_100MW_01S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_100MW_01S, 110, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_20MW_1S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_20MW_1S, 1100, buffer, 6);
        break;
    case SHT45_MODE_HIGH_PRECISION_HEATER_20MW_01S:
        SHT45_WriteRead(SHT45_MODE_HIGH_PRECISION_HEATER_20MW_01S, 110, buffer, 6);
        break;
    
    default:
    memset(buffer, 0, sizeof(uint8_t) * 6);
        break;
    }

    // Verify CRC of received data
    // ESP_LOGI("SHT45_READ_T", "Data: 0x%02X 0x%02X, CRC: 0x%02X, SHT45_CRC8(): 0x%02X", buffer[0], buffer[1], buffer[2], SHT45_CRC8(&buffer[0], 2));
    // ESP_LOGI("SHT45_READ_RH", "Data: 0x%02X 0x%02X, CRC: 0x%02X, SHT45_CRC8(): 0x%02X", buffer[3], buffer[4], buffer[5], SHT45_CRC8(&buffer[3], 2));
    // ESP_ERROR_CHECK((SHT45_CRC8(&buffer[0], 2) == buffer[2]) ? ESP_OK : ESP_FAIL);
    // ESP_ERROR_CHECK((SHT45_CRC8(&buffer[3], 2) == buffer[5]) ? ESP_OK : ESP_FAIL);
    
    *temp_raw = (uint16_t)((((uint16_t)buffer[0]) << 8) | buffer[1]);
    *hum_raw = (uint16_t)((((uint16_t)buffer[3]) << 8) | buffer[4]);

    *temp_conv = ((float)(*temp_raw) / 65535.0f) * 175.0f - 45.0f;
    *hum_conv = ((float)(*hum_raw) / 65535.0f) * 125.0f - 6.0f;

    ESP_LOGI("SHT45_READ_T", "raw: %d phy: %f forSGP:%d", *temp_raw, *temp_conv, (uint16_t)((*temp_conv + 45.0f) / 175.0f * 65535.0f));
    ESP_LOGI("SHT45_READ_RH", "raw: %d phy: %f forSGP:%d", *hum_raw, *hum_conv, (uint16_t)(*hum_conv / 100.0f * 65535.0f));
}