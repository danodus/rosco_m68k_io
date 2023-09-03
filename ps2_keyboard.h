// ps2_keyboard.h
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#ifndef PS2_KEYBOARD_H
#define PS2_KEYBOARD_H

#include "Arduino.h"

class PS2Keyboard
{

  static void pull_low(int pin);
  static void pull_high(int pin);

  static int read_bit();
  static uint8_t read_byte();
  static void write_byte(uint8_t data);

public:
  PS2Keyboard();

  static void begin(uint8_t clk_pin, uint8_t data_pin);
  static uint8_t read(bool *avail, bool *buffer_overflow);
};

#endif // PS2_KEYBOARD_H
