#pragma once
#include <Arduino.h>
#include "hardware/spi.h"
#include "pins.h"

void oled_init();
void oled_clear();
void oled_update();
void oled_char(uint8_t x, uint8_t page, char c);
void oled_text(uint8_t x, uint8_t page, const char *s);
void oled_draw_logo();
void oled_hline(uint8_t x, uint8_t y, uint8_t length);
void oled_vline(uint8_t x, uint8_t y, uint8_t length);