/*
 * @Author: calin.acr 
 * @Date: 2024-03-19 16:22:30 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-19 21:32:39
 */
#include <stdio.h>
#include "senseair_s8.h"
#include "driver/gpio.h"
#include "mbcontroller.h"

mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    {CID_INP_DATA0, (const char*)"Meter_status", (const char*)"__", MB_DEVICE_ADDR1, MB_PARAM_INPUT, 0, 16, 0, PARAM_TYPE_U16, PARAM_SIZE_U16, {.opt1=0, .opt2=0, .opt3=0}, PAR_PERMS_READ_WRITE_TRIGGER},
};
uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

void SenseairS8Init(void)
{
    void* master_handler = NULL; // Pointer to allocate interface structure
    
    mb_communication_info_t comm_info = {
        .port = UART_NUM_1,
        .mode = MB_MODE_RTU,
        .baudrate = 9600,
        .parity = MB_PARITY_NONE
    };

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_2, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialization of Modbus master for serial port
    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    if (master_handler == NULL || err != ESP_OK) {
        ESP_LOGE("MODBUS SENSEAIR", "mb controller initialization fail.");
    }

    ESP_ERROR_CHECK(mbc_master_setup((void*)&comm_info));
    ESP_ERROR_CHECK(mbc_master_set_descriptor(&device_parameters[0], num_device_parameters));

    err = mbc_master_start();
    if (err != ESP_OK) {
        ESP_LOGE("MODBUS SENSEAIR", "mb controller start fail, err=%x.", err);
    }
    uint8_t data[10];
    uint8_t type = 0;
    memset(data, 0, 10);
    mbc_master_get_parameter(CID_HOLD_DATA0, (char*)device_parameters[0].param_key, &data[0], &type);
}
