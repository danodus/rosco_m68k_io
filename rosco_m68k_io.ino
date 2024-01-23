// rosco_m68k_io.ino
// Copyright (c) 2023-2024 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2_keyboard.h"
#include "ps2_mouse.h"

#include "config.h"

#define CMD_IDENT     0xf0
#define CMD_ACK       0xff
#define CMD_NACK      0x00
#define CMD_MODE_SET  0x10

#define MODE_SCANCODE 0x00  // not supported
#define MODE_ASCII    0x01
#define MODE_PS2      0x80

PS2Keyboard keyboard;
PS2Mouse mouse;

int mode = INITIAL_MODE;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
#if ARDUINO_MEGA  
  Serial1.begin(SERIAL_BAUD_RATE);
#endif

#if KEYBOARD_ENA  
  keyboard.begin(PS2_KEYBOARD_CLK_PIN, PS2_KEYBOARD_DATA_PIN);
#endif
#if MOUSE_ENA
  mouse.begin(PS2_MOUSE_CLK_PIN, PS2_MOUSE_DATA_PIN);
#endif
}

unsigned char recv_byte(bool *avail) {
  unsigned char c = 0;
  *avail = false;
#if ARDUINO_MEGA
  if (Serial1.available()) {
    c = Serial1.read();
    *avail = true;
  }
#else
  // Arduino UNO does not have a secondary serial port
  if (Serial.available()) {
    c = Serial.read();
    *avail = true;
  }
#endif
  return c;
}

void send_byte(unsigned char c) {
#if DEBUG
  Serial.println(c, HEX);
#endif
#if ARDUINO_MEGA
  Serial1.write(c);
#else
  // Arduino UNO does not have a secondary serial port
  Serial.write(c);
#endif
}

void send_str(const char *str) {
  while(*str) {
    send_byte(*str);
    str++;
  }
}

#if SERIAL_ECHO_ENA
void echo_serial() {
  // read from port 0, send to port 1:
  if (Serial.available()) {

    int inByte = Serial.read();

    Serial1.write(inByte);
  }
}
#endif // SERIAL_ECHO_ENA

void loop() {

  bool avail, buffer_overflow;

  unsigned char c = recv_byte(&avail);
  if (avail) {
    if (c == CMD_IDENT) {
      mode = INITIAL_MODE;
      send_str("rosco_kbd");
      send_byte(mode);
      send_byte(0);   // key count -- unused
      send_byte(0);   // led count -- unused
      send_byte(0);   // capabilities -- unused
      send_byte(0);   // reserved
      send_byte(0);   // reserved 2
      send_byte(CMD_ACK);
    } else if (c == CMD_MODE_SET) {
      Serial.println("CMD SET---");
      send_byte(CMD_ACK);
      do {
        c = recv_byte(&avail);
      } while (!avail);
      if (c == MODE_ASCII || c == MODE_PS2) {
        mode = c;
        send_byte(CMD_ACK);
      } else {
        send_byte(CMD_NACK);  // unknown or unsupported mode
      }
    } else {
#if SERIAL_ECHO_ENA
      // echo to port 0
      Serial.write(c);
#endif // SERIAL_ECHO_ENA
    }
  }

#if SERIAL_ECHO_ENA
  echo_serial();
#endif // SERIAL_ECHO_ENA

#if KEYBOARD_ENA
  //
  // Keyboard
  //

  if (mode == MODE_PS2) {
    uint8_t scancode;
    scancode = keyboard.read(&avail, &buffer_overflow);
    if (buffer_overflow)
      Serial.println("Keyboard buffer overflow");
  
    if (avail) {
        send_byte('K');
        send_byte(scancode);
    }
  } else if (mode == MODE_ASCII) {
    char c;
    c = keyboard.read_ascii(&buffer_overflow);
    if (buffer_overflow)
      Serial.println("Keyboard buffer overflow");

    if (c)
      send_byte(c);
  }
#endif // KEYBOARD_ENA

#if MOUSE_ENA
  //
  // Mouse
  //

  if (mode == MODE_PS2) {
    uint8_t mstat, mx, my;
    mstat = mouse.read(&avail, &buffer_overflow);
    if (buffer_overflow)
      Serial.println("Mouse buffer overflow");
    if (avail) {
      do {
        mx = mouse.read(&avail, &buffer_overflow);
        if (buffer_overflow)
          Serial.println("Mouse buffer overflow");
      } while (!avail);
      do {
        my = mouse.read(&avail, &buffer_overflow);
        if (buffer_overflow)
          Serial.println("Mouse buffer overflow");
      } while (!avail);

      send_byte('M');
      send_byte(mstat);
      send_byte(mx);
      send_byte(my);
    }
  }
#endif // MOUSE_ENA
  
}
