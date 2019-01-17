# PIC32DDS

Generate audio-frequency signals from an MCP4822 dual 12-bit DAC
connected via SPI to a PIC32MX550F256L. SPI clock at 2MHz.
New samples sent to DAC at 44.1kHz.

* SCK2 RG6  pin 10 to MCP4822 SCK pin 3
* SDO2 RC13 pin 73 to MCP4822 SDI pin 4
* SS   RD9  pin 69 to MCP4822 CS  pin 2
* 
* LED1 RE6 pin 4
* LED2 RE7 pin 5
* LED3 RE1 pin 94
* LED4 RA7 pin 91
* LED5 RA6 pin 92

PIC32 pin numbers are for the 100-pin package.

LEDs light when the pin is pulled LOW.
