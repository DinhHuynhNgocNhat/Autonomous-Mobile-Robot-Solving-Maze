#pragma once
#include <stdint.h>

void encoder_init();
int32_t encoder_left_ticks();
int32_t encoder_right_ticks();
void encoder_reset();