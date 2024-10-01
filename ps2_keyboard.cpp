// ps2_keyboard.cpp
// Copyright (c) 2023-2024 Daniel Cliche
// SPDX-License-Identifier: MIT

// Ref.: 
//  - https://github.com/kristopher/PS2-Mouse-Arduino
//  - https://gist.github.com/mifritscher/fe8058a9ec294522a88d3d62fb9f6498

#include "ps2_keyboard.h"

#include "ps2.h"
#include "ps2_rosco.h"

#define BUFFER_SIZE 128

#define KEYMAP_SIZE 132
static const char g_keymap[] = 
// Without shift or control
"             \x09""`      q1   zsaw2  cxde43   vftr5  nbhgy6   mju78  ,kio09"
"  ./l;p-   \' [=    \x0D""] \\        \x08""  1 47   0.2568\x1B""  +3-*9      "
// With shift
"             \x09""~      Q!   ZSAW@  CXDE$#   VFTR%  NBHGY^   MJU&*  <KIO)("
"  >?L:P_   \" {+    \x0D""} |        \x08""  1 47   0.2568\x1B""  +3-*9      "
// With control
"             \x09""`      \x11""1   \x1A""\x13""\x01""\x17""2  \x03""\x18""\x04""\x05""43  \x00""\x16""\x06""\x14""\x12""5  \x0E""\x02""\x08""\x07""\x19""6   \x0D""\x0A""\x15""78  ,\x0B""\x09""\x0F""09"
"  ./\x0C"";\x10""-   \' \x1B""=    \x0D""\x1D"" \x1C""        \x08""  1 47   0.2568\x1B""  +3-*9      "
// With control and shift
"             \x09""`      \x11""1   \x1A""\x13""\x01""\x17""\x00""  \x03""\x18""\x04""\x05""43   \x16""\x06""\x14""\x12""5  \x0E""\x02""\x08""\x07""\x19""\x1E""   \x0D""\x0A""\x15""78  ,\x0B""\x09""\x0F""09"
"  ./\x0C"";\x10""\x1F""   \' [=    \x0D""] \\        \x08""  1 47   0.2568\x1B""  +3-*9      ";

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

char PS2Keyboard::read_ascii(bool *buffer_overflow) {
  static int brk = 0, modifier = 0, shift = 0, control = 0;
  for (;;) {
    // if character available
    uint8_t code;
    bool avail;
    code = read(&avail, buffer_overflow);
    if (avail) {
        if (code == 0xAA) { // BAT completion code
          continue; 
        }
        if (code == 0xF0) {
          brk = 1;
          continue;
        }
        if (code == 0xE0) {
          modifier = 1;
          continue;
        }
        if (brk) {
          if ((code == 0x12) || (code == 0x59)) {
              shift = 0;
          } else if (code == 0x14)
              control = 0;
          brk = 0;
          modifier = 0;
          continue;
        }
        if ((code == 0x12) || (code == 0x59)) {
          // left of right shift
          shift = 1;
          brk = 0;
          modifier = 0;
          continue;
        }
        if (code == 0x14) {
          // left or right control
          control = 1;
          brk = 0;
          modifier = 0;
          continue;
        }
        
        if (modifier)
            return (char)code;

        char c = 0;
        if (code < KEYMAP_SIZE)
            c = g_keymap[code + KEYMAP_SIZE * (control << 1 | shift)];
        return c;
      } else {
        // not blocking
        break;
      }
    }

    // no character available
    return 0;
}

uint8_t PS2Keyboard::read_rosco(bool *buffer_overflow) {
  static bool brk = false, modifier = false;
  for (;;) {
    // if character available
    uint8_t code;
    bool avail;
    code = read(&avail, buffer_overflow);
    if (avail) {
        if (code == 0xAA) { // BAT completion code
          continue; 
        }
        if (code == 0xF0) {
          brk = true;
          continue;
        }
        if (code == 0xE0) {
          modifier = true;
          continue;
        }        
        code = ps2_to_rosco_scancode(!brk, modifier, code);
        brk = false;
        modifier = false;
        return code;
      } else {
        // not blocking
        break;
      }
    }

    // no character available
    return 0;
}
