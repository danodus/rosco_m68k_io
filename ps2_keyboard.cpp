// ps2_keyboard.cpp
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2_keyboard.h"

#include "ps2.h"

#define BUFFER_SIZE 128
static volatile uint8_t g_buffer[BUFFER_SIZE];
static volatile uint8_t g_head, g_tail;
static volatile bool g_buffer_overflow;
static uint8_t g_clk_pin, g_data_pin;

static void clk_interrupt(void) {
  static uint8_t bitcount = 0;
  static uint8_t incoming = 0;
  static uint32_t prev_ms = 0;
  uint32_t now_ms;
  uint8_t n, val;

  val = digitalRead(g_data_pin);
  now_ms = millis();
  if (now_ms - prev_ms > 250) {
    bitcount = 0;
    incoming = 0;
  }
  prev_ms = now_ms;
  n = bitcount - 1;
  if (n <= 7) {
    incoming |= (val << n);
  }
  bitcount++;
  if (bitcount == 11) {
    uint8_t i = g_head + 1;
    if (i >= BUFFER_SIZE)
      i = 0;
    if (i != g_tail) {
      g_buffer[i] = incoming;
      g_head = i;
    } else {
      g_buffer_overflow = true;
    }
    bitcount = 0;
    incoming = 0;
  }
}

PS2Keyboard::PS2Keyboard() {
}

uint8_t PS2Keyboard::read(bool *avail, bool *buffer_overflow) {
  *avail = false;
  *buffer_overflow = g_buffer_overflow;
  g_buffer_overflow = false;

  uint8_t c, i;

  i = g_tail;
  if (i == g_head)
    return 0;
  i++;
  if (i >= BUFFER_SIZE)
    i = 0;
  c = g_buffer[i];
  g_tail = i;
  *avail = true;
  return c;
}

void PS2Keyboard::begin(int clk_pin, int data_pin) {

  g_clk_pin = clk_pin;
  g_data_pin = data_pin;

  // initialize the keyboard
  ps2_pull_high(clk_pin);
  ps2_pull_high(data_pin);
  delay(20);
  ps2_write_byte(clk_pin, data_pin, 0xff);  // send reset
  ps2_read_byte(clk_pin, data_pin);         // read ack
  delayMicroseconds(100);
  ps2_pull_high(clk_pin);
  ps2_pull_high(data_pin);   

  // attach the interrupt handler
  g_head = 0;
  g_tail = 0;
  g_buffer_overflow = false;
  attachInterrupt(digitalPinToInterrupt(clk_pin), clk_interrupt, FALLING);
}
