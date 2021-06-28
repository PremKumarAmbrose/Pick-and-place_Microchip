

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "serial_rs232.h"


// Serial & Timing Parameters
static double const Fosc              = 4000000;// Oscillator Frequency in Hz
static double       Tosc              = 1/Fosc; // Tosc in sec
static double const desired_BaudRate  = 9600;    // Desired Baud Rate in bps


// USART configuration
void init_USART(void)
{
    //From Datasheet, in case of Asynchronous Mode:
    //Desired Baud Rate = FOSC / (64 (X + 1)), if TXSTA<BRGH> = 0 or
    //Desired Baud Rate = FOSC / (16 (X + 1)), if TXSTA<BRGH> = 1 
    
    // X is the nearest integer value for the SPBRG register needed to obtain the desired baud rate
    
    //The choice depends on 
    // - Fosc 
    // - Error = (Calculated Baud Rate - Desired Baud Rate)/Desired Baud Rate
    
    // In this case with Fosc = 4 MHz and 9600 bps as desired baud rate
    
    //SPBRG = ( (Fosc / Desired Baud Rate) / 16 ) - 1
    
    SPBRG = round(((Fosc / desired_BaudRate)/16)-1); // baud rate
        
	TXSTAbits.TX9=0;   //TX 8 bits data
    TXSTAbits.TXEN=1;  //TX enable
    TXSTAbits.SYNC=0;  //Asynchronous Mode
    TXSTAbits.BRGH=1;  //High baud rate bit
    RCSTAbits.SPEN=1;  //Serial Port enable
    RCSTAbits.RX9=0;   //RX 8 bits data
    RCSTAbits.CREN=1;  //RX enable
}

void print_string(char strng[])
{
    for(int i=0; strng[i]!=NULL;i++){
        TXREG=strng[i];
        while(!TXSTAbits.TRMT);   
    }
  
}
void print_char(char chr){
    TXREG=chr;
    while(!TXSTAbits.TRMT);
}
//Add functions here 

// eg. transmit or receive a string

