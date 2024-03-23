/*
 * @Author: calin.acr 
 * @Date: 2024-02-29 18:23:15 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-21 01:53:50
 */
#include <stdio.h>
#include "string.h"
#include "oled_display.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;
spi_bus_config_t bus_cfg = {
    .sclk_io_num = OLED_SCLK_PIN,
    .mosi_io_num = OLED_MOSI_PIN,
    .miso_io_num = -1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = GDDRAM_SIZE
};
esp_lcd_panel_io_spi_config_t io_config = {
    .dc_gpio_num = OLED_DC_PIN,
    .cs_gpio_num = OLED_CS_PIN,
    .pclk_hz = OLED_SPI_SPEED,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .spi_mode = 0,
    .trans_queue_depth = 10,
    .flags.dc_low_on_data = 0,
    // .user_ctx = &disp_drv
};
esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = OLED_RESET_PIN,
    .bits_per_pixel = 1,
    .flags.reset_active_high = 0,
};

uint8_t * GDDRAM_frame = NULL;
bool isFrameChanged = false;

uint8_t page_changed[8] = { 0 };

void oled_init(void)
{
    GDDRAM_frame = heap_caps_malloc(GDDRAM_SIZE, MALLOC_CAP_DMA);
    memset(GDDRAM_frame, 0x00, GDDRAM_SIZE);


    ESP_LOGI("oled_display", "Initialize SPI bus");
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    ESP_LOGI("oled_display", "Install panel IO");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)OLED_HOST, &io_config, &io_handle));

    ESP_LOGI("oled_display", "Install SSD1309 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    // Startup sequence
    // Display off
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    // Set oscillator frequency
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xD5, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x80, NULL, 0));
    // Set multiplexer ratio
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA8, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x3F, NULL, 0));
    // Set display offset
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xD3, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x00, NULL, 0));
    // Set display start line
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x40, NULL, 0));
    // Charge pump setting
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x8D, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x14, NULL, 0));
    // Set memory addressing mode
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x20, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x02, NULL, 0));
    // Set segment re-map
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA1, NULL, 0));
    // Set COM output scan direction
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xC8, NULL, 0));
    // Set COM pins hardware configuration
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xDA, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x12, NULL, 0));
    // Set contrast control
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x81, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x05, NULL, 0));
    // Set pre-charge period
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xD9, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA1, NULL, 0));
    // Set VCOMH deselect level
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xDB, NULL, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x00, NULL, 0));
    // Deactivate scroll
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x2E, NULL, 0));
    // Output ram to display
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA4, NULL, 0));
    // Set normal/invert (0xA6/0xA7)
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA6, NULL, 0));
    // Display on
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    for (uint8_t i = 0; i < DISPLAY_HEIGHT / 8; i++)
    {
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xB0 + i, NULL, 0));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x00, NULL, 0));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x10 + i, NULL, 0));
        
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_handle, -1, &GDDRAM_frame[DISPLAY_WIDTH * i], DISPLAY_WIDTH));
        
    }
}

void oled_display_main(void)
{
    while (true)
    {
        if (isFrameChanged)
        {
            oled_flush();
        }
    }
}

void oled_flush(void)
{
    if (isFrameChanged)
    {
        // Iterate through pages
        for (uint8_t page = 0; page < 8; page++)
        {
            // Check if the current page was changed by calling oled_draw_glyph
            if (page_changed[page] == 1)
            {
                ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xB0 | (page & 0x0F), NULL, 0));
                ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x10, NULL, 0));
                ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x00, NULL, 0));

                ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_handle, -1, &GDDRAM_frame[DISPLAY_WIDTH * page], DISPLAY_WIDTH));
            }
        }
        // Reset page status
        memset(page_changed, 0, 8);
    }

}

void oled_draw_glyph(char glyph, uint8_t row, uint8_t column)
{
    // uint8_t index = glyph ? (glyph - 0x20) : (glyph);
    uint8_t index = CHAR_2_GLYPH(glyph);

    memcpy(&GDDRAM_frame[(row * DISPLAY_WIDTH) + (column * 6)], &Font6x8[index * 6], 6);
    // Signal a change of page data
    page_changed[row] = 1;
    isFrameChanged = true;
}

void oled_draw_text(char string[], uint8_t row, uint8_t column)
{
    uint8_t i = 0;

    while (*string != '\0')
    {
        oled_draw_glyph(*string, row, column + i);
        i++;
        string+=1;
    }
}
