#pragma once
#include "pico/stdlib.h"
#include "pins.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// === Function declarations ===
void buttons_init(void);
bool buttonA_pressed(void);
bool buttonB_pressed(void);
bool buttonC_pressed(void);
bool wait_for_button_release(uint pin);

// === ToF sensor enable pins ===
#define TOF_FRONT   27
#define TOF_RIGHT   24

// === I2C addresses ===
#define VL6180X_BASE_ADDRESS   0x29
#define VL6180X_LEFT_ADDRESS   0x30
#define VL6180X_FRONT_ADDRESS  0x31
#define VL6180X_RIGHT_ADDRESS  0x32

// === Public API ===
void tof_init();
void tof_update();
uint16_t tof_get_left();
uint16_t tof_get_front();
uint16_t tof_get_right();