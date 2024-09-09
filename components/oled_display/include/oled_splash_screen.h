/*
 * @Author: calin.acr 
 * @Date: 2024-09-09 20:15:32 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-09-09 21:28:15
 */

#if !defined(_OLED_SPLASH_SCREEN_H)
#define _OLED_SPLASH_SCREEN_H

// 'splash_screen', 128x64px
const char splash_screen[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xfc,  // 16
0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0xdc, 0xdc, 0xdc,  // 32
0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0x80, 0x80, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xf0, 0xf0,  // 48
0xf8, 0xf8, 0x7c, 0x3c, 0x3c, 0x7c, 0xf8, 0xf8, 0xf8, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xfc,  // 64
0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xf0, 0x80, 0x00, 0x80, 0xf8, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc,  // 80
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xe0, 0xf0, 0xf8, 0xf8, 0xfc,  // 96
0x3c, 0x3c, 0x7c, 0xf8, 0xf8, 0xf8, 0xf0, 0xc0, 0x00, 0x00, 0xe0, 0xf0, 0xf8, 0xf8, 0x7c, 0x3c,  // 112
0x3c, 0xfc, 0xf8, 0xf8, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 128
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xf0, 0xff, 0xff, 0x7f,  // 16
0x7b, 0x78, 0x7f, 0x7f, 0xff, 0xff, 0xfe, 0xf0, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,  // 32
0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x07, 0x07, 0x00, 0x04, 0x7f, 0xff, 0xff,  // 48
0xff, 0xff, 0xe0, 0xc0, 0xc0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x0f, 0x00, 0x00, 0x00, 0xff,  // 64
0xff, 0xff, 0x03, 0x07, 0x7f, 0xff, 0xff, 0xfe, 0xff, 0x7f, 0x07, 0x03, 0xff, 0xff, 0xff, 0xff,  // 80
0x00, 0x00, 0x3c, 0x3c, 0x3c, 0x3c, 0x3c, 0x3c, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xe0,  // 96
0xc0, 0xc0, 0xc0, 0xf9, 0xf9, 0xf9, 0xf9, 0x39, 0x00, 0x20, 0xe0, 0xf0, 0xf0, 0xf1, 0xcf, 0xcf,  // 112
0xcf, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 128
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, // 
0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // 
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x0f, 0x1f, 0x1f, 0x1c, 0x1c, 0x1c, 0x00, 0x00, 0x00, 0x03, // 
0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, // 
0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, // 
0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x70, 0x38, 0x1c, 0x0e, 0x1e, 0x1e, 0x3c, 0xf8, 0xf0, 0xe0, 0xc0, 0xf8, 0x78, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xe0, 0xf0, 0xfc, 0x7e, // 
0x1e, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8, 0xf8, 0xf8, 0xf0, 0x00, 0x00, 0xf0, 0xf8, 0xf8, 0xf8, // 
0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xe1, 0x7f, 0x3f, 0x7f, 0xfc, // 
0xf0, 0x80, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x70, 0x38, 0x0e, 0xc7, 0xff, 0xff, 0x0f, 0x01, 0x00, // 
0x80, 0x80, 0xe0, 0xf0, 0x38, 0x0c, 0x8c, 0xfc, 0xfc, 0x00, 0x00, 0x00, 0x80, 0xf0, 0xf8, 0x3c, // 
0x1c, 0x18, 0x18, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x20, 0x30, 0xf8, 0xf8, 0xf8, 0x00, 0x00, 0xc0, // 
0xf8, 0xfc, 0x38, 0x00, 0x00, 0x00, 0xfc, 0xfc, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x3e, 0x7f, 0x7f, 0x3e, 0x1c, 0x00, 0x81, 0xe3, 0xf3, 0xf1, 0xf8, 0xf8, 0xf1, 0xf3, 0xe3, 0xc3, // 
0x81, 0x00, 0x3e, 0x7f, 0x7f, 0x3e, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf8, 0x7e, 0x6f, 0x03, 0x00, 0x00, 0x00, 0x01, // 
0x0f, 0xff, 0xfe, 0xfc, 0x1e, 0x07, 0x01, 0x00, 0x00, 0x78, 0xff, 0xff, 0x7f, 0x0c, 0x00, 0x00, // 
0x7c, 0xff, 0x87, 0x81, 0xc1, 0x61, 0x31, 0x19, 0x0c, 0x00, 0x00, 0xfc, 0xff, 0x87, 0x00, 0x00, // 
0x80, 0xe0, 0xf8, 0x7f, 0x1f, 0x01, 0x00, 0x00, 0x80, 0xf0, 0xff, 0x7f, 0x39, 0x1c, 0x0e, 0x0f, // 
0xff, 0xff, 0xc0, 0xe0, 0x78, 0x1e, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x3c, 0x7e, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x3f, 0x7f, 0x7f, 0xff, 0xff, // 
0xff, 0x7f, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 
0x00, 0x00, 0x18, 0x38, 0xf8, 0xf9, 0xf9, 0x19, 0x18, 0x18, 0x00, 0xf8, 0xf8, 0xf8, 0x98, 0x98, // 
0x98, 0x99, 0x19, 0x19, 0x00, 0xe0, 0xf0, 0xf8, 0x38, 0x18, 0x18, 0x78, 0x78, 0x70, 0x00, 0x00, // 
0xc0, 0xf8, 0xf8, 0xf8, 0x80, 0x80, 0x80, 0xf8, 0xf8, 0xf8, 0x00, 0x00, 0xc1, 0xf9, 0xf9, 0xf9, // 
0xf9, 0xe1, 0x80, 0xe0, 0xf8, 0xf8, 0x00, 0x00, 0xc0, 0xe0, 0xf0, 0xf8, 0x38, 0x18, 0x18, 0xf8, // 
0xf8, 0xf0, 0xc0, 0x00, 0x00, 0xf8, 0xf8, 0xf8, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe0, // 
0xf0, 0x78, 0x18, 0x18, 0x38, 0xf8, 0xf8, 0xf0, 0x00, 0x00, 0xc0, 0xe0, 0xf0, 0xf8, 0x38, 0x18, // 
0x18, 0x78, 0x78, 0x30, 0x00, 0x00, 0x80, 0xf8, 0xf8, 0xf8, 0x08, 0x00, 0x80, 0xf8, 0xf8, 0xf8, // 
0x98, 0x98, 0x98, 0x98, 0x18, 0x08, 0x00, 0xf0, 0xf0, 0xf8, 0x98, 0x98, 0x98, 0xb8, 0x38, 0x10, // 
0x00, 0x00, 0x00, 0x3f, 0x3f, 0x3f, 0x01, 0x00, 0x00, 0x00, 0x3e, 0x3f, 0x3f, 0x3b, 0x31, 0x31, // 
0x31, 0x31, 0x00, 0x00, 0x0f, 0x1f, 0x3f, 0x3f, 0x30, 0x30, 0x3c, 0x1c, 0x0c, 0x04, 0x00, 0x30, // 
0x3f, 0x3f, 0x1f, 0x03, 0x01, 0x33, 0x3f, 0x3f, 0x3f, 0x01, 0x00, 0x30, 0x3f, 0x3f, 0x03, 0x01, // 
0x07, 0x1f, 0x3f, 0x3f, 0x3f, 0x01, 0x00, 0x00, 0x1f, 0x3f, 0x3f, 0x38, 0x30, 0x30, 0x3c, 0x1f, // 
0x0f, 0x07, 0x00, 0x00, 0x3e, 0x3f, 0x3f, 0x3f, 0x30, 0x30, 0x30, 0x10, 0x00, 0x06, 0x1f, 0x3f, // 
0x3f, 0x30, 0x30, 0x38, 0x3e, 0x1f, 0x0f, 0x07, 0x00, 0x00, 0x1f, 0x3f, 0x3f, 0x38, 0x30, 0x3b, // 
0x1f, 0x3f, 0x3f, 0x07, 0x00, 0x20, 0x3f, 0x3f, 0x3f, 0x01, 0x00, 0x20, 0x3f, 0x3f, 0x3f, 0x3b, // 
0x31, 0x31, 0x31, 0x31, 0x00, 0x08, 0x18, 0x39, 0x39, 0x33, 0x33, 0x3b, 0x3f, 0x1f, 0x0f, 0x00  // 
};


#endif // _OLED_SPLASH_SCREEN_H
