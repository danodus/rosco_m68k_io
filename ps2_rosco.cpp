#include "ps2_rosco.h"

static const uint8_t ps2_rosco_map[] = {
  0x00, // 00
  0x00, // 01
  0x00, // 02
  0x00, // 03
  0x3F, // 04 F3
  0x1F, // 05 F1
  0x2F, // 06 F2
  0x00, // 07
  0x00, // 08
  0x00, // 09
  0x00, // 0A
  0x00, // 0B
  0x4F, // 0C F4
  0x21, // 0D TAB
  0x42, // 0E `
  0x00, // 0F
  0x00, // 10
  0x52, // 11 LALT
  0x41, // 12 LSHIFT
  0x00, // 13
  0x51, // 14 LCTRL
  0x22, // 15 Q
  0x12, // 16 1
  0x00, // 17
  0x00, // 18
  0x00, // 19
  0x43, // 1A Z
  0x33, // 1B S
  0x32, // 1C A
  0x23, // 1D W
  0x13, // 1E 2
  0x00, // 1F
  0x00, // 20
  0x45, // 21 C
  0x44, // 22 X
  0x34, // 23 D
  0x24, // 24 E
  0x15, // 25 4
  0x14, // 26 3
  0x00, // 27
  0x00, // 28
  0x57, // 29 SPACEBAR
  0x46, // 2A V
  0x35, // 2B F
  0x26, // 2C T
  0x25, // 2D R
  0x16, // 2E 5
  0x00, // 2F
  0x00, // 30
  0x48, // 31 N
  0x47, // 32 B
  0x37, // 33 H
  0x36, // 34 G
  0x27, // 35 Y
  0x17, // 36 6
  0x00, // 37
  0x00, // 38
  0x00, // 39
  0x49, // 3A M
  0x38, // 3B J
  0x28, // 3C U
  0x18, // 3D 7
  0x19, // 3E 8
  0x00, // 3F
  0x00, // 40
  0x4A, // 41 ,
  0x39, // 42 K
  0x29, // 43 I
  0x2A, // 44 O
  0x1B, // 45 0
  0x1A, // 46 9
  0x00, // 47
  0x00, // 48
  0x4B, // 49 .
  0x4C, // 4A /
  0x3A, // 4B L
  0x3B, // 4C ;
  0x2B, // 4D P
  0x1C, // 4E -
  0x00, // 4F
  0x00, // 50
  0x00, // 51
  0x3C, // 52 '
  0x00, // 53
  0x2C, // 54 [
  0x1D, // 55 =
  0x00, // 56
  0x00, // 57
  0x31, // 58 CAPSLOCK
  0x4D, // 59 RSHIFT
  0x3E, // 5A ENTER
  0x2D, // 5B ]
  0x00, // 5C
  0x2E, // 5D '\'
  0x00, // 5E
  0x00, // 5F
  0x00, // 60
  0x00, // 61
  0x00, // 62
  0x00, // 63
  0x00, // 64
  0x00, // 65
  0x1E, // 66 BACKSPACE
  0x00, // 67
  0x00, // 68
  0x00, // 69
  0x00, // 6A
  0x00, // 6B
  0x00, // 6C
  0x00, // 6D
  0x00, // 6E
  0x00, // 6F
  0x00, // 70
  0x00, // 71
  0x00, // 72
  0x00, // 73
  0x00, // 74
  0x00, // 75
  0x11, // 76 ESC
  0x00, // 77
  0x00, // 78
  0x00, // 79
  0x00, // 7A
  0x00, // 7B
  0x00, // 7C
  0x00, // 7D
  0x00, // 7E
  0x00, // 7F
  0x00, // 80
  0x00, // 81
  0x00, // 82
  0x00, // 83
  0x00, // 84
  0x00, // 85
  0x00, // 86
  0x00, // 87
  0x00, // 88
  0x00, // 89
  0x00, // 8A
  0x00, // 8B
  0x00, // 8C
  0x00, // 8D
  0x00, // 8E
  0x00, // 8F
};

uint8_t ps2_to_rosco_scancode(bool make, bool modifier, uint8_t ps2_scancode) {
  uint8_t c = 0x00;
  if (modifier) {
    switch (ps2_scancode) {
      case 0x75: // UP
        c = 0x71;
        break;
      case 0x72: // DOWN
        c = 0x74;
        break;
      case 0x6B: // LEFT
        c = 0x72;
        break;
      case 0x74: // RIGHT
        c = 0x73;
        break;
      case 0x14: // RCTRL
        c = 0x5E;
        break;
      case 0x1F: // LMETA
        c = 0x53; // ISO key
        break;
      case 0x27: // RMETA
        c = 0x5B; // ISO key
        break;
      case 0x11: // RALT
        c = 0x5C;
        break;
    }
  } else {
    if (ps2_scancode < sizeof(ps2_rosco_map))
      c = ps2_rosco_map[ps2_scancode];
  }
  if (c && make)
    c |= 0x80;
  return c;
}