/*
 * @Author: calin.acr 
 * @Date: 2024-03-19 16:22:34 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-19 21:07:27
 */
#include "esp_modbus_common.h"
#include "esp_modbus_master.h"

enum {
    MB_DEVICE_ADDR1 = 1,
    MB_SLAVE_COUNT
};

enum {
    CID_INP_DATA0 = 0,      // Meter status
    CID_INP_DATA1,          // Alarm status
    CID_INP_DATA2,          // Output status
    CID_INP_DATA3,          // Space CO2
    CID_INP_DATA21 = 21,    // PWM output
    CID_INP_DATA25 = 25,    // Sensor type ID high
    CID_INP_DATA26 = 26,    // Sensor type ID low
    CID_INP_DATA27 = 27,    // Memory map version
    CID_INP_DATA28 = 28,    // FW version main.sub
    CID_INP_DATA29 = 29,    // Sensor ID high
    CID_INP_DATA30 = 30,    // Sensor ID low
};

enum {
    CID_HOLD_DATA0 = 0,     // Acknowledgement register
    CID_HOLD_DATA1,         // Special command register
    CID_HOLD_DATA31 = 31,   // ABC period in hours
};

extern mb_parameter_descriptor_t device_parameters[];

extern uint16_t num_device_parameters;

void SenseairS8Init(void);