// rosco_m68k_io.ino
// Copyright (c) 2023-2024 Daniel Cliche
// SPDX-License-Identifier: MIT

#include "ps2_keyboard.h"
#include "ps2_mouse.h"

#include "config.h"

// Commands
#define CMD_LED_POWRED      0x1
#define CMD_LED_POWGRN      0x2
#define CMD_LED_POWBLU      0x3
#define CMD_LED_CAPS        0x4
#define CMD_LED_DISK        0x5
#define CMD_LED_EXTRED      0x6
#define CMD_LED_EXTGRN      0x7
#define CMD_LED_EXTBLU      0x8

#define CMD_IDENT           0xf0
#define CMD_RESET           0xf1

#define CMD_MODE_SET        0x10
#define CMD_RPT_DELAY_SET   0x11
#define CMD_RPT_RATE_SET    0x12

#define CMD_MOUSE_DETECT    0x20
#define CMD_MOUSE_STRM_ON   0x21
#define CMD_MOUSE_STRM_OFF  0x22
#define CMD_MOUSE_REPORT    0x23
#define CMD_MOUSE_SET_RATE  0x24
#define CMD_MOUSE_SET_RES   0x25
#define CMD_MOUSE_SET_SCALE 0x26

#define CMD_ACK             0xff
#define CMD_NACK            0x00

#define MODE_SCAN           0x00
#define MODE_ASCII          0x01
#define MODE_PS2            0x80

PS2Keyboard keyboard;
PS2Mouse mouse;

int mode = INITIAL_MODE;
bool mouse_stream_ena = false;
uint8_t mstat = 0, mx = 0, my = 0;

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
      send_byte(CMD_ACK);
      do {
        c = recv_byte(&avail);
      } while (!avail);
      if (c == MODE_ASCII || c == MODE_SCAN || c == MODE_PS2) {
        mode = c;
        Serial.print("Mode set to ");
        if (mode == MODE_ASCII) {
          Serial.println("ASCII");
        } else if (mode == MODE_SCAN) {
          Serial.println("SCAN");
        } else {
          Serial.println("PS2");
        }
        send_byte(CMD_ACK);
      } else {
        send_byte(CMD_NACK);  // unknown or unsupported mode
      }
    } else if (c == CMD_LED_DISK) {
      send_byte(CMD_ACK);
      do {
        c = recv_byte(&avail);
      } while (!avail);
      if (c == 0x00 || c == 0x01) {
        Serial.print("Disk LED ");
        if (c == 0x00) {
          Serial.println("OFF");
        } else if (c == 0x01) {
          Serial.println("ON");
          send_byte(CMD_ACK);
        } else {
          send_byte(CMD_NACK);
        }
      }     
    } else if (c == CMD_RESET) {
      Serial.println("Reset");
      mode = MODE_ASCII;
      send_byte(CMD_ACK);
    } else if (c == CMD_MOUSE_DETECT) {
      Serial.println("Mouse detect query");
#if MOUSE_ENA
      send_byte(CMD_ACK);
#else // MOUSE_ENA
      send_byte(CMD_NACK);
#endif // MOUSE_ENA
    } else if (c == CMD_MOUSE_STRM_ON) {
      Serial.println("Mouse stream ON");
      mouse_stream_ena = true;
      send_byte(CMD_ACK);
    } else if (c == CMD_MOUSE_STRM_OFF) {
      Serial.println("Mouse stream OFF");
      mouse_stream_ena = false;
      send_byte(CMD_ACK);
    } else if (c == CMD_MOUSE_REPORT) {
#if MOUSE_ENA
      send_byte(CMD_ACK);
      send_byte(0x60);
      send_byte(mstat);
      send_byte(mstat & 0x10 ? -mx : mx);
      send_byte(mstat & 0x20 ? -my : my);
      send_byte(0x00); // TODO
      send_byte(CMD_ACK);
      mx = 0;
      my = 0;
#else // MOUSE_ENA
      send_byte(CMD_NACK);
#endif // MOUSE_ENA

    } else {
#if SERIAL_ECHO_ENA
      // echo to port 0
      Serial.write(c);
#else // SERIAL_ECHO_ENA
      Serial.println("Unknown command received!");
      send_byte(CMD_NACK);
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
  } else if (mode == MODE_SCAN) {
    uint8_t s = keyboard.read_rosco(&buffer_overflow);
    if (buffer_overflow)
      Serial.println("Keyboard buffer overflow");

    if (s)
      send_byte(s);
  }
#endif // KEYBOARD_ENA

#if MOUSE_ENA
  //
  // Mouse
  //

  if (mode == MODE_PS2 || mode == MODE_SCAN) {

    uint8_t tmp_mstat = 0, tmp_mx = 0, tmp_my = 0;

    tmp_mstat = mouse.read(&avail, &buffer_overflow);
    if (buffer_overflow)
      Serial.println("Mouse buffer overflow");
    if (avail) {
      do {
        tmp_mx = mouse.read(&avail, &buffer_overflow);
        if (buffer_overflow)
          Serial.println("Mouse buffer overflow");
      } while (!avail);
      do {
        tmp_my = mouse.read(&avail, &buffer_overflow);
        if (buffer_overflow)
          Serial.println("Mouse buffer overflow");
      } while (!avail);

      mstat = tmp_mstat;
      mx = tmp_mx;
      my = tmp_my;

      if (mode == MODE_PS2) {
        send_byte('M');
        send_byte(mstat);
        send_byte(mx);
        send_byte(my);
      } else if (mode == MODE_SCAN) {
        if (mouse_stream_ena) {
          send_byte(0x60);
          send_byte(mstat);
          send_byte(mstat & 0x10 ? -mx : mx);
          send_byte(mstat & 0x20 ? -my : my);
          send_byte(0x00); // TODO
        }
      }
    }
  }
#endif // MOUSE_ENA
  
}
