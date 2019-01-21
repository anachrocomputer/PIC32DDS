# PIC32DDS

Generate audio-frequency signals from an MCP4822 dual 12-bit DAC
connected via SPI to a PIC32MX550F256L.
SPI2 clock at 2MHz, SPI3 clock at 1MHz.
New samples sent to DAC at 44.1kHz.
SPI3 connects to an OLED display module, 128x64 pixel,
based on the SSD1306 chip.
SYNC pin set HIGH at beginning of synthesised waveform cycle and LOW
at the end.

DAC connections:

| Signal | Chip  | Name      | Pin | Chip    | Name | Pin |
|--------|-------|-----------|-----|---------|------|-----|
| SCK2   | PIC32 | RG6/SCK2  | 10  | MCP4822 | SCK  | 3   |
| MOSI2  | PIC32 | RC13/SDO2 | 73  | MCP4822 | SDI  | 4   |
| SS2    | PIC32 | RD9       | 69  | MCP4822 | CS   | 2   |

OLED display connections:

| Signal | Chip  | Name      | Pin | Chip    | Name | Pin |
|--------|-------|-----------|-----|---------|------|-----|
| SCK3   | PIC32 | RF13/SCK3 | 39  | SSD1306 | SCK  | 3   |
| MOSI3  | PIC32 | RG8/SDO3  | 12  | SSD1306 | SDA  | 4   |
| SS3    | PIC32 | RA0       | 17  | SSD1306 | CS   | 7   |
| DC     | PIC32 | RG9       | 14  | SSD1306 | DC   | 6   |
| RES    | PIC32 | RE8       | 18  | SSD1306 | RES  | 5   |

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
