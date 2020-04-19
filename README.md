# CC1101_STM32_Library
STM32 is example project and library with CC1101 on STM32CubeIDE.

I used Nucleo L432KC which i use:
  - 8MHz external oscillator
  - SPI communication protocol
  - LD3 led to check sending and receiving data
  - UART com. protocol to send to serial port

I used E07-M1101S (based on CC1101) which i use:
  - 26 MHz external oscillator
  - 433 MHz base frequency
  - GFSK modulation format
  
Connection diagram:
  - STM32 ---------------- CC1101
  - 3.3V ----------------- VCC
  - PB0 ------------------ GDO0
  - PA5 ------------------ CSN
  - PA1 ------------------ SCK
  - PA7 ------------------ MOSI
  - PA6 ------------------ MISO
  - unconnected ---------- GDO2
  - GND ------------------ GND

NOTE: GDO0 pin must be interrupt pin(GPIO_MODE_IT_FALLING) in receiver/modem mode, it must be input pin in transiver mode.

I will add some feature for future.
