/*
 * @Author: calin.acr 
 * @Date: 2024-03-15 19:23:55 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-16 01:34:59
 */

struct I2CDevMessage
{
    int32_t voc_index_value;
    int32_t nox_index_value;
    float temp_phy;
    float hum_phy;
} xI2CDevMessage;

struct PMSRecvMessage
{
    uint16_t frame_len;
    uint16_t pm10_std;
    uint16_t pm25_std;
    uint16_t pm100_std;
    uint16_t pm10_env;
    uint16_t pm25_env;
    uint16_t pm100_env;
    uint16_t particles_03um;
    uint16_t particles_05um;
    uint16_t particles_10um;
    uint16_t particles_25um;
    uint16_t particles_50um;
    uint16_t particles_100um;
    uint16_t dummy;
    uint16_t checksum;
} xPMSRecvMessage;
