// ps2_keyboard.cpp
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2_keyboard.h"

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

void PS2Keyboard::pull_low(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void PS2Keyboard::pull_high(int pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

int PS2Keyboard::read_bit() {
  while (digitalRead(g_clk_pin));
  int bit = digitalRead(g_data_pin);
  while (!digitalRead(g_clk_pin));
  return bit;
}

uint8_t PS2Keyboard::read_byte() {
  uint8_t data = 0;
  pull_high(g_clk_pin);
  pull_high(g_data_pin);
  delayMicroseconds(50);
  while (digitalRead(g_clk_pin));
  delayMicroseconds(5);   
  while (!digitalRead(g_clk_pin));   // eat start bit
  for (int i = 0; i < 8; i++)
    bitWrite(data, i, read_bit());
  read_bit();                          // parity bit
  read_bit();                          // stop bit
  pull_low(g_clk_pin);
  return data;
}

void PS2Keyboard::write_byte(uint8_t data) {
  char i;
  char parity = 1;
  pull_high(g_data_pin);
  pull_high(g_clk_pin);
  delayMicroseconds(300);
  pull_low(g_clk_pin);
  delayMicroseconds(300);
  pull_low(g_data_pin);
  delayMicroseconds(10);
  pull_high(g_clk_pin);             // start bit
  while (digitalRead(g_clk_pin));   // wait for mouse to take control of clock
  // clock is low, and we are clear to send data
  for (i = 0; i < 8; i++) {
    if (data & 0x01) {
      pull_high(g_data_pin);
    } else {
      pull_low(g_data_pin);
    }
    // wait for clock cycle
    while (!digitalRead(g_clk_pin));
    while (digitalRead(g_clk_pin));
    parity = parity ^ (data & 0x01);
    data >>= 1;
  }
  // parity
  if (parity) {
    pull_high(g_data_pin);
  } else {
    pull_low(g_data_pin);
  }
  while (!digitalRead(g_clk_pin));
  while (digitalRead(g_clk_pin));
  pull_high(g_data_pin);
  delayMicroseconds(50);
  while (digitalRead(g_clk_pin));
  while ((!digitalRead(g_clk_pin)) || (!digitalRead(g_data_pin))); // wait for mouse to switch modes
  pull_low(g_clk_pin);                                              // put a hold on the incoming data
}

void PS2Keyboard::begin(uint8_t clk_pin, uint8_t data_pin) {

  g_clk_pin = clk_pin;
  g_data_pin = data_pin;

  // initialize the mouse
  pull_high(clk_pin);
  pull_high(data_pin);
  delay(20);
  write_byte(0xff);  // send reset
  read_byte();       // read ack
  delayMicroseconds(100);
  pull_high(clk_pin);
  pull_high(data_pin);   

  // attach the interrupt handler
  g_head = 0;
  g_tail = 0;
  g_buffer_overflow = false;
  attachInterrupt(digitalPinToInterrupt(clk_pin), clk_interrupt, FALLING);
}
