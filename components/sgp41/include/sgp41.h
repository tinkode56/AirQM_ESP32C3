/*
 * @Author: calin.acr 
 * @Date: 2024-03-14 18:20:23 
 * @Last Modified by:   calin.acr 
 * @Last Modified time: 2024-03-14 18:20:23 
 */
#include "esp_err.h"
#include "driver/i2c_master.h"

#define SGP41_COMMAND_EXECUTE_CONDITIONING        0x2612U        /**< execute conditioning command */
#define SGP41_COMMAND_MEASURE_RAW                 0x2619U        /**< measure raw command */
#define SGP41_COMMAND_EXECUTE_SELF_TEST           0x280EU        /**< execute self test command */
#define SGP41_COMMAND_TURN_HEATER_OFF             0x3615U        /**< turn heater off command */
#define SGP41_COMMAND_GET_SERIAL_ID               0x3682U        /**< get serial id command */
#define SGP41_COMMAND_SOFT_RESET                  0x0006U        /**< soft reset command */

extern i2c_master_dev_handle_t sgp_device_handle;

void SGP41_Init(i2c_master_bus_handle_t *mbh);
void SGP41_Read(uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay);
void SGP41_Write(uint16_t reg, uint8_t *data, uint16_t len);
void SGP41_ReadWithParam(uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay, uint8_t *output, uint16_t output_len);
void SGP41_ExecuteConditioning(uint16_t *voc_raw);
void SGP41_MeasureRawSignals(uint16_t temp_raw, uint16_t hum_raw, uint16_t *voc_raw, uint16_t *nox_raw);