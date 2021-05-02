
/*
 * File:   main.c
 * Author: Sourav Gupta
 * By:- circuitdigest.com
 * Created on May 10, 2018, 1:26 PM
 * This program will drive a servo motor.
 */
 
// PIC16F877A Configuration Bit Settings
 
// 'C' source line config statements
 
// CONFIG
/*
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
*/ 
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config OSCS = OFF       // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config BORV = 20        // Brown-out Reset Voltage bits (VBOR set to 2.0V)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3H
#pragma config CCP2MUX = OFF    // CCP2 Mux bit (CCP2 input/output is multiplexed with RB3)

// CONFIG4L
#pragma config STVR = OFF       // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will not cause RESET)
#pragma config LVP = OFF        // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-001FFFh) not code protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-001FFFh) not write protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-001FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from Table Reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from Table Reads executed in other blocks)
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <xc.h>
#include "init_PIC.h"
#include "serial_rs232.h"  


// Serial & Timing Parameters
static double const Fosc              = 4000000;// Oscillator Frequency in Hz

static double       Tosc              = 1/Fosc; // Tosc in sec
static double const desired_BaudRate  = 9600;    // Desired Baud Rate in bps
static _Bool   New_char_RX = false;
int lenth_of_array = 30;
char testarray [30];
char testarray[] = "This is prem test case";


#define X_Pick_1 3  //Pick and Place (X,Y)position of C
#define Y_Pick_1 4
#define X_Place_1 17
#define Y_Place_1 5

#define X_Pick_2 3  //Pick and Place (X,Y)position of A
#define Y_Pick_2 10
#define X_Place_2 10
#define Y_Place_2 5

#define X_Pick_3 3  //Pick and Place (X,Y)position of B
#define Y_Pick_3 15
#define X_Place_3 16
#define Y_Place_3 14


#define Pick_Angle_1 0       //Pick angle of C
#define Place_Angle_1 270    //Place angle of C

#define Pick_Angle_2 90      //Pick angle of A   
#define Place_Angle_2 180    //Place angle of A

#define Pick_Angle_3 90      //Pick angle of B
#define Place_Angle_3 270    //Place angle of B






/*
 Hardware related definition
 */
 
#define _XTAL_FREQ 200000000 //Crystal Frequency, used in delay
#define speed 1 // Speed Range 10 to 1  10 = lowest , 1 = highest
#define clockwise 0 // clockwise direction macro
#define anti_clockwise 1 // anti clockwise direction macro
 
 
/*
 *Application related function and definition
 */
 
void system_init (void); // This function will initialise the ports.
void X_axis (char direction);
void Y_axis (char direction);
void Z_axis (char direction);
void Twister (char direction);
void Tweezer (void);
void ms_delay(unsigned int val);
void pickandplace_sequence(void);
void __interrupt() Rx_char_USART (void);


/*
 * main function starts here
 */
 
 
void main(void)
{
    unsigned char RX_Char = ' ';  // used for echo of the received char
    unsigned char test_C = ' ';remKumarAmbrose/Pick-and-place_Microchip
    
    init_PORTS();           // PORTS configuration
    init_USART();           // USART module configuration
    init_interrupts();      // Interrupt configuration (only INT on RX USART enabled)
       
    ei();                   // enable all interrupts
    for(int i=0;i<2;i++){pickandplace_sequence();}
    
    while(1){               // inifinte loop
        
        if (New_char_RX)    // sent "Hello World!" if a new char is RX on USART
        {
            RX_Char = RCREG;            // Read received char
            serial_tx_char(RX_Char);    // Echo received char
            serial_tx_char('\r');
            serial_tx_char('\n');
            
            // TX string char by char   
            serial_tx_char('H');
            serial_tx_char('e');
            serial_tx_char('l');
            serial_tx_char('l');
            serial_tx_char('o');
            serial_tx_char(' ');
            serial_tx_char('W');
            serial_tx_char('o');
            serial_tx_char('r');
            serial_tx_char('l');
            serial_tx_char('d');
            serial_tx_char('!');
            serial_tx_char('\r');       
            serial_tx_char('\n'); 

            for(int i=0; i<lenth_of_array; i++){
                test_C = testarray[i];
                serial_tx_char(test_C);
                }
            
            serial_tx_char('\r');       
            serial_tx_char('\n');
            
            //LATBbits.LATB0 = !LATBbits.LATB0;       //toggle led on RB0
            New_char_RX = false;                    //reset monitor variable
        }                   // end if
    } 
        
}


/*System Initialising function to set the pin direction Input or Output*/
 


 
/* This method will drive the motor in half-drive mode using direction input */
 
-
void X_axis (char direction){
    if (direction == anti_clockwise){
        PORTB = 0b00000011;
        ms_delay(speed);
        PORTB = 0b00000110;
        ms_delay(speed);
        PORTB = 0b00001100;
        ms_delay(speed);
        PORTB = 0b00001001;
        ms_delay(speed);
        PORTB = 0b00000011;
        ms_delay(speed);
    }
    if (direction == clockwise){
        PORTB = 0b00001001;
        ms_delay(speed);
        PORTB = 0b00001100;
        ms_delay(speed);
        PORTB = 0b00000110;
        ms_delay(speed);
        PORTB = 0b00000011;
        ms_delay(speed);
        PORTB = 0b00001001;
        ms_delay(speed);
    }
    
}

void Y_axis (char direction){
    if (direction == anti_clockwise){
        PORTB = 0b00110000;
        ms_delay(speed);
        PORTB = 0b01100000;
        ms_delay(speed);
        PORTB = 0b11000000;
        ms_delay(speed);
        PORTB = 0b10010000;
        ms_delay(speed);
        PORTB = 0b00110000;
        ms_delay(speed);
    }
    if (direction == clockwise){
        PORTB = 0b10010000;
        ms_delay(speed);
        PORTB = 0b11000000;
        ms_delay(speed);
        PORTB = 0b01100000;
        ms_delay(speed);
        PORTB = 0b00110000;
        ms_delay(speed);
        PORTB = 0b10010000;
        ms_delay(speed);
    }
    
}

void Z_axis (char direction){
    if (direction == anti_clockwise){
        PORTD = 0b00000011;
        ms_delay(speed);
        PORTD = 0b00000110;
        ms_delay(speed);
        PORTD = 0b00001100;
        ms_delay(speed);
        PORTD = 0b00001001;
        ms_delay(speed);
        PORTD = 0b00000011;
        ms_delay(speed);
    }
    if (direction == clockwise){
        PORTD = 0b00001001;
        ms_delay(speed);
        PORTD = 0b00001100;
        ms_delay(speed);
        PORTD = 0b00000110;
        ms_delay(speed);
        PORTD = 0b00000011;
        ms_delay(speed);
        PORTD = 0b00001001;
        ms_delay(speed);
    }
    
}

void Twister (char direction){
    if (direction == anti_clockwise){
        PORTD = 0b00110000;
        ms_delay(speed);
        PORTD = 0b11000000;
        ms_delay(speed);
        PORTD = 0b10010000;
        ms_delay(speed);
        PORTD = 0b00110000;
        ms_delay(speed);
    }
    if (direction == clockwise){
        PORTD = 0b10010000;
        ms_delay(speed);
        PORTD = 0b01100000;
        ms_delay(speed);
        PORTD = 0b00110000;
        ms_delay(speed);
        PORTD = 0b10010000;
        ms_delay(speed); 
    }
    
}
/*This method will create required delay*/
 
void ms_delay(unsigned int val)
{
     unsigned int i,j;
        for(i=0;i<val;i++)
            for(j=0;j<1650;j++);
}



void pickandplace_sequence(void)
{
    //// Component C //////////////////////////////////
    //Tweezer open 5 units wide
    for(int i = 0; i<(X_Pick_1); i++){X_axis(clockwise);}
    for(int i = 0; i<(Y_Pick_1); i++){Y_axis(clockwise);}
    for(int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer close 4 units wide
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up
    for(int i = 0; i<(X_Place_1-X_Pick_1); i++){X_axis(clockwise);}
    for(int i = 0; i<(Y_Place_1-Y_Pick_1); i++){Y_axis(clockwise);}
    for(int i = 0; i<((360-Place_Angle_1)/3.6); i++){Twister(anti_clockwise);}
    for( int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer open 5 units wide
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up



    //// Component A //////////////////////////////////
    for(int i = 0; i<(X_Place_1-X_Pick_2); i++){X_axis(anti_clockwise);}
    for(int i = 0; i<(Y_Place_1-Y_Pick_2); i++){Y_axis(clockwise);}
    for(int i = 0; i<((Place_Angle_1-Pick_Angle_2)/3.6); i++){Twister(clockwise);}
    for(int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer close 4 units wide
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up
    for(int i = 0; i<(X_Pick_2-X_Place_2); i++){X_axis(clockwise);}
    for(int i = 0; i<(Y_Pick_2-Y_Place_2); i++){Y_axis(anti_clockwise);}
    for(int i = 0; i<((Pick_Angle_2-Place_Angle_2)/3.6); i++){Twister(clockwise);}
    for(int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer open 5 units wide
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up
    
    

    //// Component B //////////////////////////////////
    for(int i = 0; i<(X_Place_2-X_Pick_3); i++){X_axis(anti_clockwise);}
    for(int i = 0; i<(Y_Place_2-Y_Pick_3); i++){Y_axis(clockwise);}
    for(int i = 0; i<((Place_Angle_2-Pick_Angle_3)/3.6); i++){Twister(anti_clockwise);}
    for( int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer close 2 units wide
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up
    for(int i = 0; i<(X_Pick_3-X_Place_3); i++){X_axis(clockwise);}
    for(int i = 0; i<(Y_Pick_3-Y_Place_3); i++){Y_axis(anti_clockwise);}
    for(int i = 0; i<((Pick_Angle_3-Place_Angle_3)/3.6); i++){Twister(clockwise);}
    for(int i = 0;i<3; i++){Z_axis(clockwise);}  //down
    //Tweezer open 3 units wide 
    for(int i = 0; i<3; i++){Z_axis(anti_clockwise);}  //up
     

    ////Return to Initial position //////////////////////////////////
    for(int i = 0; i<(X_Place_3); i++){X_axis(anti_clockwise);}
    for(int i = 0; i<(Y_Place_3); i++){Y_axis(anti_clockwise);}

}

void __interrupt() Rx_char_USART(void)  // Interrupt function
{     
    if(PIE1bits.RCIE && PIR1bits.RCIF)  // only process USART RX char interrupts
        {
            New_char_RX = true;         // set flag variable
            PIR1bits.RCIF = 0;          // clear this interrupt condition
        }                               // end IF
}                                       // end function
