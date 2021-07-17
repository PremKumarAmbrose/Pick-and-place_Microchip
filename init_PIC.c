


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "init_PIC.h"



#define TMR1H_0     0xFA	// Fine tuned Timer1 registers (Ton = 1.5 msec => set position 0°)
#define TMR1L_0     0x24	// 

#define TMR0H_set   0xB2	// Fine tuned Timer0 registers (PWM period)
#define TMR0L_set   0x04	// 

static uint8_t new_TMR1H = TMR1H_0; 	//new pulse duration (i.e. new position of the servo)
static uint8_t new_TMR1L = TMR1L_0;	


void init_PORTS(void)          //PORTS configurations
{
    //TRISBbits.RB0 = 1;     // RB0 as input
    //TRISBbits.RB1 = 1;
    TRISD = 0x00;
    TRISA = 0x00;
    TRISB = 0X03;  // RB0 and RB1 as input //TRISB=0X0f; // Enable the 4 LSB as I/P & 4 MSB as O/P
    PORTA = 0x00; //0xF0;
    //PORTD = 0x00; //0x0F;
    //PORTB= 0X0F;
    
    
    // PORTC
    LATC    = 0;        //All pins off
    PORTC   = 0;
    TRISCbits.RC7 = 1;  //All pins as input (default), RC7 as Input (Usart RX)
    TRISCbits.RC6 = 0;  //RC6 as output (Usart TX)
    TRISCbits.RC2 = 0;
    
    return;
}

//--------------------------------------------------
void init_Timers(void){
    T0CONbits.T08BIT = 0;   //16-bit
	T0CONbits.T0CS   = 0;	//internal instruction cycle clock (timer)
	T0CONbits.PSA    = 1;	//No prescaler
	T0CONbits.TMR0ON = 0;	//Timer 0 disabled 
    
    
	TMR0H = TMR0H_set;       //set TMR0H
	TMR0L =	TMR0L_set;       //set TMR0L
	
    //Timer 1
    T1CONbits.RD16	  = 1;  //16-bit
	T1CONbits.T1CKPS1 = 0;  //No prescaler
	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1OSCEN = 0;  //Timer 1 ext oscillator disabled
	T1CONbits.TMR1CS  = 0;  //Internal clock (Fosc/4)
	T1CONbits.TMR1ON  = 0;  //Timer 1 disabled
	
    //set intitial position = 0° 
	TMR1H = TMR1H_0;
    TMR1L = TMR1L_0;
		   
    return;
    
}


void init_interrupts(void)       // Interrupts configuration 
{
    PIR1bits.RCIF = 0;          //Clear RCIF Interrupt Flag
	PIE1bits.RCIE = 1;          //Set RCIE Interrupt Enable (USART RX)
	INTCONbits.PEIE	= 1;        //Enable peripheral interrupts
    
    PIE1bits.TMR1IE = 1;        // enable interrupt on Timer1 overflows
    INTCONbits.TMR0IE = 1;      // enable interrupt on Timer0 overflow
    
    INTCON2=0x00;		/* Set Interrupt on falling Edge*/
    //INTCON =0x00;
    INTCON3=0x00;
    INTCONbits.INT0IF=0;	/* Clear INT0IF flag*/
    INTCONbits.INT0IE=1;
    INTCON3bits.INT1IF=0;
    INTCON3bits.INT1IE=1;
    //Global Interrupt enabled in main ( ei(); )
    return;
}
//--------------------------------------------------
