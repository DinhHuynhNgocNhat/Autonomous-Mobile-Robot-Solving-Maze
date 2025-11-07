#pragma once
#include "pico/stdlib.h"
#include "pins.h"

// === Function declarations ===
void sensors_init(void);
bool buttonA_pressed(void);
bool buttonB_pressed(void);
bool buttonC_pressed(void);
bool wait_for_button_release(uint pin);