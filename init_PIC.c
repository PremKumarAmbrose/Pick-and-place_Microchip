


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "init_PIC.h"

void init_PORTS(void)          //PORTS configurations
{
    //TRISBbits.RB0 = 1;     // RB0 as input
    //TRISBbits.RB2 = 1;
    TRISD = 0x00;
    TRISA = 0x00;
    TRISB = 0X0F;  //TRISB=0X0f; // Enable the 4 LSB as I/P & 4 MSB as O/P
    PORTA = 0x00; //0xF0;
    PORTD = 0x00; //0x0F;
    //PORTB= 0X0F;
    
    
    // PORTC
    LATC    = 0;        //All pins off
    PORTC   = 0;
    TRISCbits.RC7 = 1;  //All pins as input (default), RC7 as Input (Usart RX)
    TRISCbits.RC6 = 0;  //RC6 as output (Usart TX)
    
    return;
}
//--------------------------------------------------


void init_interrupts(void)       // Interrupts configuration 
{
    PIR1bits.RCIF = 0;          //Clear RCIF Interrupt Flag
	PIE1bits.RCIE = 1;          //Set RCIE Interrupt Enable (USART RX)
	INTCONbits.PEIE	= 1;        //Enable peripheral interrupts
    
    
    
    INTCON2=0x00;		/* Set Interrupt on falling Edge*/
    INTCONbits.INT0IF=0;	/* Clear INT0IF flag*/
    INTCONbits.INT0IE=1;
    //Global Interrupt enabled in main ( ei(); )
    return;
}
//--------------------------------------------------
