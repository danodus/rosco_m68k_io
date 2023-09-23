// rosco_m68k_io.ino
// Copyright (c) 2023 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2_keyboard.h"
#include "ps2_mouse.h"

#include "config.h"

PS2Keyboard keyboard;
PS2Mouse mouse;

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

#if SERIAL_ECHO_ENA
void echo_serial() {
  // read from port 1, send to port 0:
  if (Serial1.available()) {

    int inByte = Serial1.read();

    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {

    int inByte = Serial.read();

    Serial1.write(inByte);
  }
}
#endif // SERIAL_ECHO_ENA

void loop() {

  bool avail, buffer_overflow;

#if SERIAL_ECHO_ENA
  echo_serial();
#endif // SERIAL_ECHO_ENA

#if KEYBOARD_ENA
  //
  // Keyboard
  //

  uint8_t scancode;
  scancode = keyboard.read(&avail, &buffer_overflow);
  if (buffer_overflow)
    Serial.println("Keyboard buffer overflow");
 
  if (avail) {
    send_byte('K');
    send_byte(scancode);
  }
#endif // KEYBOARD_ENA

#if MOUSE_ENA
  //
  // Mouse
  //

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
#endif // MOUSE_ENA
  
}
