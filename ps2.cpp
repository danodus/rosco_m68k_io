// ps2.cpp
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2.h"

void ps2_pull_low(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void ps2_pull_high(int pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

int ps2_read_bit(int clk_pin, int data_pin) {
  while (digitalRead(clk_pin));
  int bit = digitalRead(data_pin);
  while (!digitalRead(clk_pin));
  return bit;
}

uint8_t ps2_read_byte(int clk_pin, int data_pin) {
  uint8_t data = 0;
  ps2_pull_high(clk_pin);
  ps2_pull_high(data_pin);
  delayMicroseconds(50);
  while (digitalRead(clk_pin));
  delayMicroseconds(5);   
  while (!digitalRead(clk_pin));           // eat start bit
  for (int i = 0; i < 8; i++)
    bitWrite(data, i, ps2_read_bit(clk_pin, data_pin));
  ps2_read_bit(clk_pin, data_pin);         // parity bit
  ps2_read_bit(clk_pin, data_pin);         // stop bit
  ps2_pull_low(clk_pin);
  return data;
}

void ps2_write_byte(int clk_pin, int data_pin, uint8_t data) {
  char i;
  char parity = 1;
  ps2_pull_high(data_pin);
  ps2_pull_high(clk_pin);
  delayMicroseconds(300);
  ps2_pull_low(clk_pin);
  delayMicroseconds(300);
  ps2_pull_low(data_pin);
  delayMicroseconds(10);
  ps2_pull_high(clk_pin);             // start bit
  while (digitalRead(clk_pin));   // wait for mouse to take control of clock
  // clock is low, and we are clear to send data
  for (i = 0; i < 8; i++) {
    if (data & 0x01) {
      ps2_pull_high(data_pin);
    } else {
      ps2_pull_low(data_pin);
    }
    // wait for clock cycle
    while (!digitalRead(clk_pin));
    while (digitalRead(clk_pin));
    parity = parity ^ (data & 0x01);
    data >>= 1;
  }
  // parity
  if (parity) {
    ps2_pull_high(data_pin);
  } else {
    ps2_pull_low(data_pin);
  }
  while (!digitalRead(clk_pin));
  while (digitalRead(clk_pin));
  ps2_pull_high(data_pin);
  delayMicroseconds(50);
  while (digitalRead(clk_pin));
  while ((!digitalRead(clk_pin)) || (!digitalRead(data_pin))); // wait for mouse to switch modes
  ps2_pull_low(clk_pin);                                       // put a hold on the incoming data
}
