// ps2.h
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#ifndef PS2_H
#define PS2_H

#include "Arduino.h"

void ps2_pull_low(int pin);
void ps2_pull_high(int pin);

uint8_t ps2_read_byte(int clk_pin, int data_pin);
void ps2_write_byte(int clk_pin, int data_pin, uint8_t data);

#endif // PS2_H
