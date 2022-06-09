#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

#include "ws2812b.h"

#include <stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

#define PIC32_SYS_FREQ 48000000ul // 48 million Hz
#define PIC32_DESIRED_BAUD 230400 // Baudrate for RS232


int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    LATAbits.LATA4 = 0;
    
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    
    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX
    
    ws2812b_setup();
     
    __builtin_enable_interrupts();
    
    char m[50];
    
    float hue1 = 0;
    float hue2 = 60;
    float hue3 = 120;
    float hue4 = 180;
    float hue5 = 240;
    
    LATBbits.LATB6 = 0;
    TMR2 = 0;
    
    while (TMR2 < 2400) {
        ;
    }
    
    while (1) {
        wsColor c1 = HSBtoRGB(hue1, 1, 0.1);
        wsColor c2 = HSBtoRGB(hue2, 1, 0.1);
        wsColor c3 = HSBtoRGB(hue3, 1, 0.1);
        wsColor c4 = HSBtoRGB(hue4, 1, 0.1);
        wsColor c5 = HSBtoRGB(hue5, 1, 0.1);
        wsColor cArray[5] =lor1, c2, c3, c4, c5};
        
        hue1++;
        hue2++;
        hue3++;
        hue4++;
        hue5++;
        
        if (hue1 > 360) {
            hue1 = 0;
        }
        if (hue2 > 360) {
            hue2 = 0;
        }
        if (hue3 > 360) {
            hue3 = 0;
        }
        if (hue4 > 360) {
            hue4 = 0;
        }
        if (hue5 > 360) {
            hue5 = 0;
        }
        
        ws2812b_setColor(cArray, 5);
        
        TMR2 = 0;
        while (TMR2 < 60000) {
        ;
        }
        
    }
}