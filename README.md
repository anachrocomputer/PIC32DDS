# PIC32DDS

Generate audio-frequency signals from an MCP4822 dual 12-bit DAC
connected via SPI to a PIC32MX550F256L.
SPI clock at 2MHz.
New samples sent to DAC at 44.1kHz.
SYNC pin set HIGH at beginning of synthesised waveform cycle and LOW
at the end.

DAC connections:

| Signal | Chip  | Name      | Pin | Chip    | Name | Pin |
|--------|-------|-----------|-----|---------|------|-----|
| SCK    | PIC32 | RG6/SCK2  | 10  | MCP4822 | SCK  | 3   |
| MOSI   | PIC32 | RC13/SDO2 | 73  | MCP4822 | SDI  | 4   |
| SS     | PIC32 | RD9       | 69  | MCP4822 | CS   | 2   |

Digital output:

* SYNC RD0 pin 72

Debugging LEDs on dev board:

* LED1 RE6 pin 4
* LED2 RE7 pin 5
* LED3 RE1 pin 94
* LED4 RA7 pin 91
* LED5 RA6 pin 92

PIC32 pin numbers are for the 100-pin package.

LEDs light when the pin is pulled LOW.
