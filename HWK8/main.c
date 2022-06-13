#include <sys/attribs.h> // __ISR macro
#include <xc.h>          // processor SFR definitions
#include "i2c_master_noint.h"
#include "ssd1306.h"
#include "font.h"
#include <stdio.h>



// DEVCFG0
#pragma config DEBUG = OFF       // disable debugging
#pragma config JTAGEN = OFF      // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF         // disable flash write protect
#pragma config BWP = OFF         // disable boot write protect
#pragma config CP = OFF          // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF  // disable secondary oscillator
#pragma config IESO = OFF     // disable switching clocks
#pragma config POSCMOD = HS   // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD       // disable clock switch and FSCM
#pragma config WDTPS = PS1048576    // use largest wdt
#pragma config WINDIS = OFF         // use non-window mode wdt
#pragma config FWDTEN = OFF         // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0     // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF  // allow multiple reconfigurations

void writeI2C(unsigned char address, unsigned char reg, unsigned char value);
unsigned char readI2C(unsigned char address, unsigned char address2,
                      unsigned char reg, int ack);

void readUART1(char * message, int maxLength);
void writeUART1(const char * string);

char m[100];

int main() {

  __builtin_disable_interrupts(); // disable interrupts while initializing
                                  // things

  // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
  __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

  // 0 data RAM access wait states
  BMXCONbits.BMXWSDRM = 0x0;

  // enable multi vector interrupts
  INTCONbits.MVEC = 0x1;

  // disable JTAG to get pins back
  DDPCONbits.JTAGEN = 0;

  // B4 is input
  TRISBbits.TRISB4 = 1;

  // A4 is output
  TRISAbits.TRISA4 = 0;

  // A4 is low
  LATAbits.LATA4 = 0;
  
  U1RXRbits.U1RXR = 0b0001; //  B6 = U1RX
  RPB7Rbits.RPB7R = 0b0001; //  B7 = U1TX
  
  // turn on UART3 without an interrupt
  U1MODEbits.BRGH = 0; // set baud to PIC32_DESIRED_BAUD
  U1BRG = ((48000000 / 230400) / 16) - 1;

  U1MODEbits.PDSEL = 0;
  U1MODEbits.STSEL = 0;

  // configure TX & RX pins as output & input pins
  U1STAbits.UTXEN = 1;
  U1STAbits.URXEN = 1;

  // enable the UART
  U1MODEbits.ON = 1;
  U1MODEbits.UEN = 0;
  
    i2c_master_setup();

  
    ssd1306_setup();


  __builtin_enable_interrupts();

  
  
//  
  ssd1306_clear();
  
  unsigned char value;

  
////////////////////////////////////// Set output pins
  
  int i = 0;
  
  while (1) 
  {

        _CP0_SET_COUNT(0); // reset core timer
        LATAINV = 0b10000; // toggle pin A4

    
//    drawChar(1, 1, 1);
        
        for (i = 0; i < 100; i++)
        {
    
        sprintf(m, "ME433 %d!", i);
        
        drawString(50, 16, m);
    
//    ssd1306_drawPixel(1, 1, 1);
        ssd1306_update();

        while (_CP0_GET_COUNT() < 6000000) 
        {
            ;
        } // 0.25s delay

   
//    value = readI2C(0b01000000, 0x09, 0b01000001, 1);
////    
//    value &= 1;
//  
//    
//    if (value == 0) {
//      writeI2C(0x40, 0x09, 0xFF); // turn on pin GP7
//        0;
//    }
//    if (value == 1) {
//      writeI2C(0x40, 0x09, 0x00); // turn off pin GP7
//        0;
     }
  }
}

// Read from UART1
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void readUART1(char *message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART3
void writeUART1(const char *string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}


void writeI2C(uint8_t devAddress, uint8_t regAddress, uint8_t data) 
{
    
  i2c_master_start();
  i2c_master_send(devAddress); // send address + write bit
  i2c_master_send(regAddress);     // send command register address
  i2c_master_send(data);   // send value
  i2c_master_stop();
  
}

unsigned char readI2C(uint8_t devAddress, uint8_t regAddress,
                      uint8_t readAddress, int ack) 
{
    
  unsigned char value;
  
  i2c_master_start();
  
  i2c_master_send(devAddress); // send address + read bit
  i2c_master_send(regAddress);     // send command register address
  i2c_master_restart();     // send a restart
  
  i2c_master_send(readAddress);
  
  value = i2c_master_recv(); // receive byte from device
  char m[100];
  writeUART1(m);       // call on writeUART1 to send PIC a message
  i2c_master_ack(ack); // send acknowledge bit
  
  i2c_master_stop();
  
  return value;
  
}