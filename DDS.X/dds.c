/* dds --- generate audio via MCP4822 SPI DAC on PIC32 dev board       */
/* Copyright (c) 2019 John Honniball. All rights reserved              */

/*
 * Created: 2019-01-17 23:17
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// PIC32MX360F512L Configuration Bit Settings
 
// 'C' source line config statements
 
// DEVCFG3
// USERID = No Setting
 
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)
#pragma config UPLLIDIV = DIV_4         // 
#pragma config UPLLEN = ON              // 
 
// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
 
// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // 
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
 
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
 
#include <xc.h>
#include <sys/attribs.h>
 
#define LED1        LATEbits.LATE6
#define LED2        LATEbits.LATE7
#define LED3        LATEbits.LATE1
#define LED4        LATAbits.LATA7
#define LED5        LATAbits.LATA6

// Size of 128x64 OLED screen
#define MAXX 128
#define MAXY 64
#define MAXROWS 8

// Co-ord of centre of screen
#define CENX (MAXX / 2)
#define CENY (MAXY / 2)

// SSD1306 command bytes
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

// The frame buffer, 1024 bytes
unsigned char Frame[MAXROWS][MAXX];

volatile uint32_t MilliSeconds = 0;

volatile uint32_t PhaseAcc = 0;
volatile uint32_t PhaseInc = 0;
uint16_t Sinbuf[4096];

volatile int SPINbytes = 0;
volatile uint8_t *SPIBuf = NULL;
volatile int SPIDummyReads = 0;

/* dally --- CPU busy-loop for crude time delay */

static void dally(const int loops)
{
    volatile int dally;
    
    for (dally = 0; dally < loops; dally++)
            ;
}


/* delayms --- busy-wait delay for given number of milliseconds */

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}

void __ISR(_TIMER_4_VECTOR, ipl4) Timer4Handler(void) 
{    
    static int flag = 0;
    int i;
    
    LATAINV = _LATA_LATA4_MASK; // Toggle RA4 P7 pin 6 (22050Hz)
    
    LATDCLR = _LATD_LATD9_MASK;   // Assert SS on RD9
    
    i = PhaseAcc >> 20;
    PhaseAcc += PhaseInc;
    
    SPI2BUF = (0 << 15) | (1 << 13) | (1 << 12) | Sinbuf[i];
    
    if (i < 2048)
    {
        LATDSET = _LATD_LATD0_MASK;     // Assert SYNC
    }
    else
    {
        LATDCLR = _LATD_LATD0_MASK;     // De-assert SYNC
    }
    
    if (flag > 31)
    {
        PR4 = 907;
        flag = 0;
    }
    else
    {
        PR4 = 906;
        flag++;
    }
    
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 interrupt flag
    IEC1SET = _IEC1_SPI2RXIE_MASK;  // Enable SPI2 interrupt
    IFS0CLR = _IFS0_T4IF_MASK;  // Clear Timer 4 interrupt flag
}

void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void) 
{
    MilliSeconds++;
    
    //LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (500Hz))
    
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
}

void __ISR(_SPI_2_VECTOR, ipl3) SPI2Handler(void) 
{
    volatile uint32_t junk;
    
    junk = SPI2BUF;
    LATDSET = _LATD_LATD9_MASK;   // De-assert SS on RD9
    
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 interrupt flag
    IEC1CLR = _IEC1_SPI2RXIE_MASK;  // Disable SPI2 interrupt
}

void __ISR(_SPI_3_VECTOR, ipl1) SPI3Handler(void) 
{
    volatile uint32_t junk;
    
    if (IFS2 & _IFS2_SPI3RXIF_MASK)
    {
        junk = SPI3BUF;
        SPIDummyReads--;
        
        IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
        
        if (SPIDummyReads == 0)
        {
            LATASET = _LATA_LATA0_MASK;

            IEC2CLR = _IEC2_SPI3RXIE_MASK;  // Disable SPI3 Rx interrupt
        }
    }
    else if (IFS2 & _IFS2_SPI3TXIF_MASK)
    {
        SPI3BUF = *SPIBuf++;        // Transmit next byte
        SPINbytes--;
        
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        
        if (SPINbytes == 0)
        {
            IEC2CLR = _IEC2_SPI3TXIE_MASK;  // Disable SPI3 Tx interrupt
        }
    }
}

static void UART1_begin(const int baud)
{
    /* Configure PPS pins */
    RPE5Rbits.RPE5R = 3;    // U1Tx on pin 3, RPE5
    U1RXRbits.U1RXR = 10;   // U1Rx on pin 6, RPC1
    
    /* Configure USART1 */
    U1MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U1STAbits.UTXEN = 1;    // Enable Tx
    U1STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U1BRG = (40000000 / (baud * 16)) - 1;
    
    U1MODESET = _U1MODE_ON_MASK;      // Enable USART1
}

static void UART2_begin(const int baud)
{
    /* Configure PPS pins */
    RPG0Rbits.RPG0R = 1;    // U2Tx on pin 90, RPG0
    
    /* Configure USART2 */
    U2MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U2STAbits.UTXEN = 1;    // Enable Tx
    U2STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U2BRG = (40000000 / (baud * 16)) - 1;
    
    U2MODESET = _U2MODE_ON_MASK;      // Enable USART2
}

static void UART3_begin(const int baud)
{
    /* Configure PPS pins */
    RPF1Rbits.RPF1R = 1;    // U3Tx on pin 88, RPF1
    
    /* Configure USART3 */
    U3MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U3STAbits.UTXEN = 1;    // Enable Tx
    U3STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U3BRG = (40000000 / (baud * 16)) - 1;
    
    U3MODESET = _U3MODE_ON_MASK;      // Enable USART3
}

static void UART4_begin(const int baud)
{
    /* Configure PPS pins */
    RPD4Rbits.RPD4R = 2;    // U4Tx on pin 81, RPD4
    
    /* Configure USART4 */
    U4MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U4STAbits.UTXEN = 1;    // Enable Tx
    U4STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U4BRG = (40000000 / (baud * 16)) - 1;
    
    U4MODESET = _U4MODE_ON_MASK;      // Enable USART4
}

static void UART5_begin(const int baud)
{
    /* Configure PPS pins */
    RPD12Rbits.RPD12R = 4;  // U5Tx on pin 79, RPD12
    
    /* Configure USART5 */
    U5MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U5STAbits.UTXEN = 1;    // Enable Tx
    U5STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U5BRG = (40000000 / (baud * 16)) - 1;
    
    U5MODESET = _U5MODE_ON_MASK;      // Enable USART5
}

static void ADC_begin(void)
{    
    AD1CON1bits.FORM = 0; // Integer
    AD1CON1bits.SSRC = 7; // Auto convert after sampling
    AD1CON1bits.ASAM = 0; // Sampling begins when SAMP bit is set
    AD1CON1bits.SAMP = 0;
    
    AD1CON2bits.VCFG = 0;  // Vdd/Vss references
    AD1CON2bits.CSCNA = 0; // Do not scan inputs
    AD1CON2bits.ALTS = 0;  // Always use MUX A
    AD1CON2bits.SMPI = 0;  // Interrupt on every conversion
    
    AD1CON3bits.ADRC = 0;   // Peripheral bus clock
    AD1CON3bits.SAMC = 15;  // Auto-sample 15 Tad
    AD1CON3bits.ADCS = 127; // Slow clock
    
    AD1CHSbits.CH0SA = 6; // Mux to AN6
    AD1CHSbits.CH0NA = 0; // Negative input is Vr-    
    
    ANSELBbits.ANSB6 = 1;   // RB6, AN6, pin 26, P1-37 analog
    TRISBbits.TRISB6 = 1;   // RB6, AN6, pin 26, P1-37 input
    
    AD1CON1SET = _AD1CON1_ON_MASK;
}


/* analogRead --- Arduino-like function to read an analog input pin */

uint16_t analogRead(const int chan)
{    
    AD1CHSbits.CH0SA = chan;
    
    AD1CON1SET = _AD1CON1_SAMP_MASK;    // Start sampling, then conversion
    
    while (AD1CON1bits.SAMP)        // Wait for sampling to complete
        ;
    
    while (AD1CON1bits.DONE == 0)   // Wait for conversion to complete
        ;
    
    return (ADC1BUF0);
}

static void SPI2_begin(const int baud)
{
    /* Configure SPI2 */
    // SCK2 on pin 10, RG6, P1 pin 32
    SDI2Rbits.SDI2R = 0;   // SDI2 on RPD3, pin 78
    RPC13Rbits.RPC13R = 6; // SDO2 on RPC13, pin 73, P7 pin 16
    
    SPI2BRG = (20000000 / baud) - 1;
    SPI2CONbits.MSTEN = 1;  // Master mode
    SPI2CONbits.MODE16 = 1; // 16-bit mode
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI2CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISDbits.TRISD9 = 0;   // RD9 pin 69, P7 pin 12 as output for SS
    LATDSET = _LATD_LATD9_MASK;   // De-assert SS for SPI2
    
    IPC8bits.SPI2IP = 3;          // SPI2 interrupt priority 3
    IPC8bits.SPI2IS = 1;          // SPI2 interrupt sub-priority 1
    IFS1CLR = _IFS1_SPI2TXIF_MASK;  // Clear SPI2 Tx interrupt flag
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 Rx interrupt flag
    
    SPI2CONbits.ON = 1;
}

static void SPI3_begin(const int baud)
{    
    /* Configure SPI3 */
    // SCK3 on pin 39, RF13, P1 pin 15
    SDI3Rbits.SDI3R = 0;   // SDI3 on RPD2, pin 77
    RPG8Rbits.RPG8R = 14;  // SDO3 on RPG8, pin 12, P1 pin 28
    
    SPI3BRG = (20000000 / baud) - 1;
    SPI3CONbits.MSTEN = 1;  // Master mode
    SPI3CONbits.MODE16 = 0; // 8-bit mode
    SPI3CONbits.MODE32 = 0;
    SPI3CONbits.CKE = 1;
    SPI3CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI3CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISAbits.TRISA0 = 0;   // RA0 pin 17, P1 pin 24 as output for SS
    LATASET = _LATA_LATA0_MASK;   // De-assert SS for SPI3
    
    IPC12bits.SPI3IP = 1;          // SPI3 interrupt priority 1
    IPC12bits.SPI3IS = 1;          // SPI3 interrupt sub-priority 1
    IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    
    SPINbytes = 0;
    SPIDummyReads = 0;
    SPIBuf = NULL;
    
    SPI3CONbits.ON = 1;
}


bool SPIwrite(uint8_t *buf, const int nbytes)
{
    if (SPIDummyReads != 0)     // SPI tranmission still in progress?
    {
        return (false);
    }
    
    if ((nbytes <= 0) || (buf == NULL))
    {
        return (false);
    }
    
    LATACLR = _LATA_LATA0_MASK;   // Assert SS for SPI3
    
    SPIBuf = buf;
    SPINbytes = nbytes;
    SPIDummyReads = nbytes;
    
    SPI3BUF = *SPIBuf++;          // Transmit first byte
    SPINbytes--;
    
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    IEC2SET = _IEC2_SPI3RXIE_MASK;  // Enable SPI3 Rx interrupt
    
    if (SPINbytes > 0)
    {
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        IEC2SET = _IEC2_SPI3TXIE_MASK;  // Enable SPI3 Tx interrupt
    }
    
    return (true);
}

int SPIbytesPending(void)
{
    return (SPIDummyReads);
}

void DDS_SetFreq(const int freq)
{
    PhaseInc = 97348 * freq;  // 97391.548662132 = 4294967296 / 44100
}


/* oledData --- send a data byte to the OLED by fast hardware SPI */

inline void oledData(const uint8_t d)
{
    static char data[2];
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 1;  // DC HIGH
    
    data[0] = d;
    SPIwrite(data, 1);
}


/* oledCmd --- send a command byte to the OLED by fast hardware SPI */

inline void oledCmd(const uint8_t c)
{
    static char cmd[2];
    
    while (SPIbytesPending() > 0)
        ;
    
    LATGbits.LATG9 = 0;  // DC LOW
    
    cmd[0] = c;
    SPIwrite(cmd, 1);
}


/* updscreen --- update the physical screen from the buffer */

static void updscreen(void)
{
// This function contains an eight-way unrolled loop. In the Arduino
// IDE, the default GCC optimisation switch is -Os, which optimises
// for space. No automatic loop unrolling is done by the compiler, so
// we do it explicitly here to save a few microseconds.
//  long int before, after;
//  unsigned char r, c;
    uint8_t *p;
    int i;

    oledCmd(SSD1306_COLUMNADDR);
    oledCmd(0);   // Column start address (0 = reset)
    oledCmd(MAXX - 1); // Column end address (127 = reset)

    oledCmd(SSD1306_PAGEADDR);
    oledCmd(0); // Page start address (0 = reset)
    oledCmd(7); // Page end address

//  before = micros ();

    p = &Frame[0][0];

    for (i = 0; i < ((MAXROWS * MAXX) / 8); i++) {
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
        oledData(*p++);
    }

/*
    The slow way...
    for (r = 0; r < MAXROWS; r++) {
        for (c = 0; c < MAXX; c++) {
            oledData(Frame[r][c]);
        }
    }
*/

//  after = micros ();
  
//  Serial.print (after - before);
//  Serial.println ("us updscreen");
}


/* OLED_begin --- initialise the 128x64 OLED */

void OLED_begin(void)
{
    /* Configure I/O pins on PIC32 */
    TRISEbits.TRISE8 = 0;     // RE8, pin 18, P1 pin 22, as output for RES
    TRISGbits.TRISG9 = 0;     // RG9, pin 14, P1 pin 26, as output for DC

    LATEbits.LATE8 = 1;       // RES pin HIGH initially
    LATGbits.LATG9 = 1;       // DC pin HIGH initially

    /* Start configuring the SSD1306 OLED controller */
    delayms(1);
    LATEbits.LATE8 = 0;     // Hardware reset for 10ms
    delayms(10);
    LATEbits.LATE8 = 1;

    // Init sequence for 128x64 OLED module
    oledCmd(SSD1306_DISPLAYOFF);                    // 0xAE
    oledCmd(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    oledCmd(0x80);                                  // the suggested ratio 0x80
    oledCmd(SSD1306_SETMULTIPLEX);                  // 0xA8
    oledCmd(0x3F);
    oledCmd(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    oledCmd(0x0);                                   // no offset
    oledCmd(SSD1306_SETSTARTLINE | 0x0);            // line #0
    oledCmd(SSD1306_CHARGEPUMP);                    // 0x8D
    oledCmd(0x14);
    oledCmd(SSD1306_MEMORYMODE);                    // 0x20
    oledCmd(0x00);                                  // 0x0 act like ks0108
    oledCmd(SSD1306_SEGREMAP | 0x1);
    oledCmd(SSD1306_COMSCANDEC);
    oledCmd(SSD1306_SETCOMPINS);                    // 0xDA
    oledCmd(0x12);
    oledCmd(SSD1306_SETCONTRAST);                   // 0x81
    oledCmd(0xCF);
    oledCmd(SSD1306_SETPRECHARGE);                  // 0xd9
    oledCmd(0xF1);
    oledCmd(SSD1306_SETVCOMDETECT);                 // 0xDB
    oledCmd(0x40);
    oledCmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    oledCmd(SSD1306_NORMALDISPLAY);                 // 0xA6

    oledCmd(SSD1306_DISPLAYON); // Turn on OLED panel
}


/* greyFrame --- clear entire frame to checkerboard pattern */

void greyFrame(void)
{
    int r, c;

    for (r = 0; r < MAXROWS; r++)
    {
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r][c] = 0xaa;
            Frame[r][c + 1] = 0x55;
        }
    }
}


void main(void)
{
    static uint8_t pixels[] = {0x03, 0x0c, 0x30, 0xc0};
    char buf[32];
    int i;
    double delta;
    uint16_t ana;
    
    /* Configure tri-state registers*/
    TRISEbits.TRISE6 = 0;   // LED1 as output
    TRISEbits.TRISE7 = 0;   // LED2 as output
    TRISEbits.TRISE1 = 0;   // LED3 as output
    TRISAbits.TRISA7 = 0;   // LED5 as output
    TRISAbits.TRISA6 = 0;   // LED5 as output
    
    TRISAbits.TRISA4 = 0;   // RA4 P7 pin 6 as output (timer toggle)
    TRISDbits.TRISD0 = 0;   // RD0 P7 pin 14 as output (SYNC signal)
    
    UART1_begin(9600);
    UART2_begin(9600);
    UART3_begin(9600);
    UART4_begin(9600);
    UART5_begin(9600);

    ADC_begin();
    
    SPI2_begin(2000000);
    SPI3_begin(1000000);
    
    RPD8Rbits.RPD8R = 12; // OC1 on P7 pin 10 (LED PWM)
    
    /* Configure Timer 3 for 10-bit PWM */
    T3CONbits.TCKPS = 6;        // Timer 3 prescale: 64
    
    TMR3 = 0x00;                // Clear Timer 3 counter
    PR3 = 1023;                 // PWM range 0..1023 (10 bits)
    
    T3CONbits.ON = 1;           // Enable Timer 3
    
    OC1CONbits.OCTSEL = 1;      // Source: Timer 3
    OC1CONbits.OCM = 6;         // PWM mode
    
    OC1RS = 256;
    
    OC1CONbits.ON = 1;          // Enable OC1 PWM
            
    /* Configure Timer 1 */
    T1CONbits.TCKPS = 0;        // Timer 1 prescale: 1
    
    TMR1 = 0x00;                // Clear Timer 1 counter
    PR1 = 39999;                // Interrupt every 40000 ticks (1ms)
    
    T1CONbits.ON = 1;           // Enable Timer 1
    
    /* Configure Timer 4 */
    T4CONbits.TCKPS = 0;        // Timer 4 prescale: 1
    
    TMR4 = 0x00;                // Clear Timer 4 counter
    PR4 = 906;                  // Interrupt every 907 ticks (44100Hz)
    
    T4CONbits.ON = 1;           // Enable Timer 4
    
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
    
    IPC1bits.T1IP = 2;          // Timer 1 interrupt priority 2
    IPC1bits.T1IS = 1;          // Timer 1 interrupt sub-priority 1
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
    IEC0SET = _IEC0_T1IE_MASK;  // Enable Timer 1 interrupt
    
    IPC4bits.T4IP = 4;          // Timer 4 interrupt priority 4
    IPC4bits.T4IS = 1;          // Timer 4 interrupt sub-priority 1
    IFS0CLR = _IFS0_T4IF_MASK;  // Clear Timer 4 interrupt flag
    IEC0SET = _IEC0_T4IE_MASK;  // Enable Timer 4 interrupt
    
    delta = (2.0 * M_PI) / 4096.0;
    
    for (i = 0; i < 4096; i++)
    {
        Sinbuf[i] = (sin(delta * (double)i) * 2047.0) + 2048.0;
    }
    
    __asm__("EI");              // Global interrupt enable
    
    OLED_begin();
    
    greyFrame();
    
    updscreen();
    
    while(1)
    {
        U1TXREG = 'A';
        DDS_SetFreq(440);
        SPIwrite(pixels, sizeof (pixels));
        
        LED1 = 0;
        LED2 = 1;
        LED3 = 1;
        LED4 = 1;
        LED5 = 1;
        
        OC1RS = 0;
        
        delayms(500);
        
        ana = analogRead(6);
        
        sprintf(buf, "%dms %d\r\n", millis(), ana);
        
        for (i = 0; buf[i] != '\0'; i++)
        {
            while (U2STAbits.UTXBF) // Wait while Tx buffer full
                ;
            
            U2TXREG = buf[i];
        }
        
        DDS_SetFreq(440 * 2);
        
        LED1 = 1;
        LED2 = 0;
        
        OC1RS = 128;
        
        delayms(500);
        
        U3TXREG = 'C';
        DDS_SetFreq(440 * 4);
        
        LED2 = 1;
        LED3 = 0;
        
        OC1RS = 256;
        
        delayms(500);
        
        U4TXREG = 'D';
        DDS_SetFreq(440 * 8);
        
        LED3 = 1;
        LED4 = 0;
        
        OC1RS = 512;
        
        delayms(500);
        
        U5TXREG = 'E';
        DDS_SetFreq(220);
        
        LED4 = 1;
        LED5 = 0;
        
        OC1RS = 1023;
        
        delayms(500);
    }
}

