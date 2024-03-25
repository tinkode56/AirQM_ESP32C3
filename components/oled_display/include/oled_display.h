/*
 * @Author: calin.acr 
 * @Date: 2024-02-29 18:06:56 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-25 17:29:59
 */

#if !defined(_OLED_DISPLAY_H)
#define _OLED_DISPLAY_H

#include <stdio.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
// #include "lvgl.h"

#define OLED_SCLK_PIN   GPIO_NUM_4
#define OLED_MOSI_PIN   GPIO_NUM_6
#define OLED_CS_PIN     GPIO_NUM_7
#define OLED_DC_PIN     GPIO_NUM_5
#define OLED_RESET_PIN  GPIO_NUM_10

#define OLED_HOST   SPI2_HOST

#define OLED_SPI_SPEED  2000000

#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

#define GDDRAM_SIZE DISPLAY_WIDTH * DISPLAY_HEIGHT / 8

#define CHARS_PER_LINE  DISPLAY_WIDTH / 6
#define LINES_PER_FRAME DISPLAY_HEIGHT / 8

#define CHAR_2_GLYPH(c)  (c) ? ((c) - 0x20) : (c)

// Frame buffer
extern uint8_t * GDDRAM_frame;

extern spi_bus_config_t bus_cfg;
extern esp_lcd_panel_io_handle_t io_handle;
extern esp_lcd_panel_io_spi_config_t io_config;
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_dev_config_t panel_config;

extern const char Font6x8[];

extern uint8_t page_changed[8];
extern bool isFrameChanged;

// Functions
void oled_init(void);
void oled_display_main(void);
void oled_flush(void);
// 21 characters per line, 8 lines
void oled_draw_glyph(char glyph, uint8_t row, uint8_t column);
void oled_draw_text(char string[], uint8_t row, uint8_t column);

#endif // _OLED_DISPLAY_H
