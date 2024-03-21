/*
 * @Author: calin.acr 
 * @Date: 2024-03-19 16:22:30 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-21 01:32:22
 */

#include "senseair_s8.h"
#include "esp_log.h"

mb_param_request_t s8_request = {
    .command = 0,
    .slave_addr = MB_DEVICE_S8,
    .reg_start = 0,
    .reg_size = 0
};

void SenseairS8_Init(void)
{
    void* master_handler = NULL; // Pointer to allocate interface structure

    mb_communication_info_t comm_info = {
    .port = SENSEAIR_UART_PORT,        // Serial port number
    .mode = MB_MODE_RTU,        // Modbus mode of communication (MB_MODE_RTU or MB_MODE_ASCII)
    .baudrate = 9600,           // Modbus communication baud rate
    .parity = MB_PARITY_NONE    // parity option for serial port
    };

    // 1. Initialization of Modbus master for serial port
    ESP_ERROR_CHECK(mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler));
    // 2. Setup of MB controller 
    ESP_ERROR_CHECK(mbc_master_setup((void*)&comm_info));
    // 3. Configure UART port pins
    ESP_ERROR_CHECK(uart_set_pin(SENSEAIR_UART_PORT, SENSEAIR_TX_PIN, SENSEAIR_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));
    // 4. Start MB controller
    ESP_ERROR_CHECK(mbc_master_start());
    // 5. Set MB controller descriptor
    // ESP_ERROR_CHECK(mbc_master_set_descriptor(&device_parameters[0], num_device_parameters));
}

void S8_ReadCO2(uint16_t *co2)
{
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = SPACE_CO2;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, co2);
}

void S8_ReadStatus(uint16_t *status)
{
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = METER_STATUS;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, status);
}

void S8_ReadCO2andStatus(uint16_t *co2, uint16_t *status)
{
    // uint8_t response[8] = { 0 };
    uint16_t response[4] = { 0 };
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = METER_STATUS;
    s8_request.reg_size = 4;
    // Send the request
    mbc_master_send_request(&s8_request, &response);
    
    // Update values
    *status = response[0];
    *co2 = response[3];
    // ESP_LOGI("S8", "DATA: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7]);
    ESP_LOGI("S8", "DATA: IR1 0x%04x, IR2 0x%04x, IR3 0x%04x, IR4 0x%04x", response[0], response[1], response[2], response[3]);
}

uint8_t S8_BgCalibration(uint8_t cal_type)
{
    uint16_t ack_clear_value = 0;
    uint16_t ack_read_value = 0;
    uint16_t bgcal_cmd = 0x7C00;

    // Step 1. Clear ACK register
    // Prepare request structure
    s8_request.command = S8_CMD_WSR;
    s8_request.reg_start = ACK_REG;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, &ack_clear_value);

    // Step 2. Write command to start BgCal
    // Prepare request structure
    s8_request.command = S8_CMD_WSR;
    s8_request.reg_start = SPECIAL_CMD;
    s8_request.reg_size = 1;
    // Append calibration type to command
    bgcal_cmd |= cal_type;
    // Send the request
    mbc_master_send_request(&s8_request, &bgcal_cmd);

    // Step 3. Wait for lamp cycle
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Step 4. Check ACK register
    // Prepare request structure
    s8_request.command = S8_CMD_RHR;
    s8_request.reg_start = ACK_REG;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, &ack_read_value);

    // Return calibration result (1 - OK, 0 - FAIL)
    if (cal_type == CO2_BG_CAL)
    {
        return CHECK_CO2_BGCAL(ack_read_value);
    }
    else
    {
        return CHECK_CO2_N_BGCAL(ack_read_value);
    }
}

void S8_ReadABC(uint16_t *abc_value)
{
    // Prepare request structure
    s8_request.command = S8_CMD_RHR;
    s8_request.reg_start = ABC_PERIOD;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, abc_value);
}

void S8_SetABC(uint16_t *abc_value)
{
    // Prepare request structure
    s8_request.command = S8_CMD_WSR;
    s8_request.reg_start = ABC_PERIOD;
    s8_request.reg_size = 1;
    // Send the request
    mbc_master_send_request(&s8_request, abc_value);
}

void S8_ReadSensorTypeID(uint32_t *value)
{
    uint16_t res[2] = { 0 };
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = S_TYPE_ID_H;
    s8_request.reg_size = 2;
    // Send the request
    mbc_master_send_request(&s8_request, &res);

    *value = (uint32_t)(res[0] << 16) | res[1];
}

void S8_ReadMemMapVers(uint16_t *value)
{
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = MEMMAP_VER;
    s8_request.reg_size = 2;
    // Send the request
    mbc_master_send_request(&s8_request, value);
}

void S8_ReadFWVers(uint16_t *value)
{
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = FW_VER;
    s8_request.reg_size = 2;
    // Send the request
    mbc_master_send_request(&s8_request, value);
}

void S8_ReadSensorID(uint32_t *value)
{
    uint16_t res[2] = { 0 };
    // Prepare request structure
    s8_request.command = S8_CMD_RIR;
    s8_request.reg_start = S_ID_H;
    s8_request.reg_size = 2;
    // Send the request
    mbc_master_send_request(&s8_request, &res);

    *value = (uint32_t)(res[0] << 16) | res[1];
}
