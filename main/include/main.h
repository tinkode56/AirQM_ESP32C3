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