// ps2_keyboard.h
// Copyright (c) 2023-2024 Daniel Cliche
// SPDX-License-Identifier: MIT

#ifndef PS2_KEYBOARD_H
#define PS2_KEYBOARD_H

#include "Arduino.h"

class PS2Keyboard
{
public:
  PS2Keyboard();

  static void begin(int clk_pin, int data_pin);
  static uint8_t read(bool *avail, bool *buffer_overflow);
  static char read_ascii(bool *buffer_overflow);
};

#endif // PS2_KEYBOARD_H
