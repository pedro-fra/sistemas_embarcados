#ifndef SSD1306_H
#define SSD1306_H

#include "driver/i2c.h"
#include "esp_err.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

esp_err_t ssd1306_init(void);
esp_err_t ssd1306_clear_screen(void);
esp_err_t ssd1306_display_text(uint8_t line, const char* text);
esp_err_t ssd1306_update_display(void);

#endif