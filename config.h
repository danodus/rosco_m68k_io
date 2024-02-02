#define ARDUINO_MEGA 1      // 0 for Arduino Uno, 1 for Arduino Mega

// Serial port
#define SERIAL_BAUD_RATE 115200

// Enabled features
#if ARDUINO_MEGA
#define SERIAL_ECHO_ENA 1   // only required when connected to the primary rosco_m68k serial port
#else
#define SERIAL_ECHO_ENA 0   // must be 0 (not available on the Arduino Uno)
#endif
#define KEYBOARD_ENA 1
#define MOUSE_ENA 1

#define INITIAL_MODE MODE_ASCII // MODE_ASCII, MODE_PS2 or MODE_EMUTOS

// Pinout
#define PS2_MOUSE_CLK_PIN 2
#define PS2_KEYBOARD_CLK_PIN 3
#define PS2_MOUSE_DATA_PIN 4
#define PS2_KEYBOARD_DATA_PIN 5

