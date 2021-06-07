


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "init_PIC.h"

void init_PORTS(void)          //PORTS configurations
{
    TRISB = 0x00;     // PORT B as output port
    TRISD = 0x00;
    PORTB = 0x0F;
    PORTD = 0x0F;
    
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
                                //Global Interrupt enabled in main ( ei(); )
    return;
}
//--------------------------------------------------
