
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
char testarray[] = " ";

int displacement_X, displacement_Y, rotation;

int X_Pick, X_Place, Y_pick, Y_Place, Pick_Angle, Place_Angle, X_diff=0, Y_diff=0, Angle_diff=0;
char X_Pick_dir,X_Place_dir, Y_Pick_dir, Y_Place_dir, Rot_Pick_dir, Rot_Place_dir,;


///*******************Specific Parameters for each component*********************///

#define X_Pick_A 3  //Pick and Place Parameters of A
#define Y_Pick_A 4
#define Pick_Angle_A 0       
char X_Pick_dir_A = clockwise;
char Y_Pick_dir_A = clockwise;
char Rot_Pick_dir_A = clockwise;

#define X_Place_A 17
#define Y_Place_A 5
#define Place_Angle_A 270    
char X_Place_dir_A = clockwise;
char Y_Place_dir_A = clockwise;
char Rot_Place_dir_A = anti_clockwise;

#define X_Pick_B 3  //Pick and Place Parameters of B
#define Y_Pick_B 10
#define Pick_Angle_B 90 
char X_Pick_dir_B = anti_clockwise;
char Y_Pick_dir_B = clockwise;
char Rot_Pick_dir_B = clockwise;

#define X_Place_B 10
#define Y_Place_B 5     
#define Place_Angle_B 180
char X_Place_dir_B = clockwise;
char Y_Place_dir_B = anti_clockwise;
char Rot_Place_dir_B = anti_clockwise;    




#define X_Pick_C 3  //Pick and Place Parameters of C
#define Y_Pick_C 15
#define Pick_Angle_C 90 
char X_Pick_dir_C = anti_clockwise;
char Y_Pick_dir_C = clockwise;
char Rot_Pick_dir_C = anti_clockwise;


#define X_Place_C 16
#define Y_Place_C 14     
#define Place_Angle_C 270
char X_Place_dir_C = clockwise;
char Y_Place_dir_C = anti_clockwise;
char Rot_Place_dir_C = clockwise;    

#define X_Pick_D 0  //Pick and Place Parameters of D
#define Y_Pick_D 0
#define Pick_Angle_D 0
char X_Pick_dir_D;
char Y_Pick_dir_D;
char Rot_Pick_dir_D;

#define X_Place_D 0
#define Y_Place_D 0
#define Place_Angle_D 0
char X_Place_dir_D;
char Y_Place_dir_D;
char Rot_Place_dir_D;    


#define X_diff 0
#define Y_diff 0
#define angle_diff 0


#define abs(x) ((x) > 0 ? (x) : -(x))



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
void pick_and_place(char Component);
void return_to_initial(void);
void __interrupt() Rx_char_USART (void);
void Z_axis_and_Tweezer(void);


/*
 * main function starts here
 */
 
 
void main(void)
{
    unsigned char RX_Char = ' ';  // used for echo of the received char
    unsigned char test_C = ' ';
    
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



void pick_and_place(char Component)
{
    //// Component C //////////////////////////////////
    //Tweezer open 5 units wide
    if(Component=A)
    {
        X_Pick=X_Pick_A;
        Y_Pick=Y_Pick_A;
        X_Pick_dir=X_Pick_dir_A;
        Y_Pick_dir=Y_Pick_dir_A;
        Pick_Angle=Pick_Angle_A;
        Rot_Pick_dir=Rot_Pick_dir_A;

        X_Place=X_Place_A;
        Y_Place=Y_Place_A;
        X_Place_dir=X_Place_dir_A;
        Y_Place_dir=Y_Place_dir_A;
        Place_Angle=Place_Angle_A;
        Rot_Place_dir=Rot_Place_dir_A;
    }
    else if(Component=B){
        X_Pick=X_Pick_B;
        Y_Pick=Y_Pick_B;
        X_Pick_dir=X_Pick_dir_B;
        Y_Pick_dir=Y_Pick_dir_B;
        Pick_Angle=Pick_Angle_B;
        Rot_Pick_dir=Rot_Pick_dir_B;

        X_Place=X_Place_B;
        Y_Place=Y_Place_B;
        X_Place_dir=X_Place_dir_B;
        Y_Place_dir=Y_Place_dir_B;
        Place_Angle=Place_Angle_B;
        Rot_Place_dir=Rot_Place_dir_B;
    }
    else if(Component=C){
        X_Pick=X_Pick_C;
        Y_Pick=Y_Pick_C;
        X_Pick_dir=X_Pick_dir_C;
        Y_Pick_dir=Y_Pick_dir_C;
        Pick_Angle=Pick_Angle_C;
        Rot_Pick_dir=Rot_Pick_dir_C;

        X_Place=X_Place_C;
        Y_Place=Y_Place_C;
        X_Place_dir=X_Place_dir_C;
        Y_Place_dir=Y_Place_dir_C;
        Place_Angle=Place_Angle_C;
        Rot_Place_dir=Rot_Place_dir_C;
    }
    else if(Component=D){
        X_Pick=X_Pick_D;
        Y_Pick=Y_Pick_D;
        X_Pick_dir=X_Pick_dir_D;
        Y_Pick_dir=Y_Pick_dir_D;
        Pick_Angle=Pick_Angle_D;
        Rot_Pick_dir=Rot_Pick_dir_D;

        X_Place=X_Place_D;
        Y_Place=Y_Place_D;
        X_Place_dir=X_Place_dir_D;
        Y_Place_dir=Y_Place_dir_D;
        Place_Angle=Place_Angle_D;
        Rot_Place_dir=Rot_Place_dir_D;
    }
    else{
        //do nothing
    }

    //displacement_X = abs(X_Pick-X_diff);
    //displacement_Y = abs(Y_Pick-Y_diff);
    //rotation  = 0;
    //X_direction = Y_direction = rotation_direction = clockwise;
    for(int i = 0; i<(abs(X_Pick-X_diff)); i++){X_axis(X_Pick_dir);}
    for(int i = 0; i<(abs(Y_Pick-Y_diff)); i++){Y_axis(Y_Pick_dir);}
    for(int i = 0; i<((Angle_diff-Pick_Angle)/3.6); i++){Twister(Rot_Pick_dir);}
    //pickandplace();
    //ultrasonic check while loop
        Z_axis_and_Tweezer();
        //Tweezer close 4 units wide

    //displacement_X = abs(X_Place-X_Pick);
    //displacement_Y = abs(Y_Place-Y_Pick);
    rotation = abs((360-Place_Angle)/3.6);
    for(int i = 0; i<(abs(X_Place-X_diff)); i++){X_axis(X_Place_dir);}
    for(int i = 0; i<(abs(Y_Place-Y_diff)); i++){Y_axis(Y_Place_dir);}
    for(int i = 0; i<((Pick_Angle-Place_Angle)/3.6); i++){Twister(Rot_Place_dir);}
    //pickandplace();
        Z_axis_and_Tweezer();
        //Tweezer open 5 units wide
    X_diff = X_Place;
    Y_diff = Y_Place;
    Angle_diff= Place_Angle;

}

/*void pickandplace_2(void)
    //// Component A //////////////////////////////////
    displacement_X = abs(X_Pick_2-X_diff);
    displacement_Y = abs(Y_Pick_2-Y_diff);
    rotation = abs((Place_Angle_1-Pick_Angle_2)/3.6);
    X_direction = anti_clockwise;
    rotation_direction= clockwise;
    pickandplace();
    //ultrasonic check while loop
        Z_axis_and_Tweezer();
        //Tweezer close 4 units wide

    displacement_X = abs(X_Place_2-X_diff);
    displacement_Y = abs(Y_Place_2-Y_diff);
    rotation = abs((Pick_Angle_2-Place_Angle_2)/3.6);
    X_direction = clockwise;
    Y_direction = anti_clockwise;
    pickandplace();
        Z_axis_and_Tweezer();
        //Tweezer open 5 units wide

    
    
void pickandplace_3(void);
    //// Component B //////////////////////////////////
    displacement_X = abs(X_Pick_3-X_diff);
    displacement_Y = abs(Y_Pick_3-Y_diff);
    rotation = abs((Place_Angle_2-Pick_Angle_3)/3.6);
    X_direction = rotation_direction= anti_clockwise;
    Y_direction = clockwise;
    pickandplace();
    //ultrasonic check while loop
        Z_axis_and_Tweezer();
        //Tweezer close 2 units wide

    displacement_X = abs(X_Place_3-X_diff);
    displacement_Y = abs(Y_Place_3-Y_diff);
    rotation = abs((Pick_Angle_3-Place_Angle_3)/3.6);
    X_direction = rotation_direction= clockwise;
    Y_direction = anti_clockwise;
    pickandplace();
        Z_axis_and_Tweezer();
        //Tweezer open 3 units wide 
     
*/

void return_to_initial(void);    
    ////Return to Initial position //////////////////////////////////
    for(int i = 0; i<X_diff; i++){X_axis(anti_clockwise);}
    for(int i = 0; i<Y_diff; i++){Y_axis(anti_clockwise);}
    for(int i = 0; i<Angle_diff; i++){Twister(anti_clockwise);}

}
/*
void pickandplace(){

    for(int i = 0; i<displacement_X; i++){X_axis(X_direction);}
    for(int i = 0; i<displacement_Y; i++){Y_axis(Y_direction);}
    for(int i = 0; i<rotation; i++){Twister(rotation_direction);}

}*/

void Z_axis_and_Tweezer(){
     for(int i = 0;i<15; i++){Z_axis(clockwise);}  //down
     //Tweezer actions, units refer sequence
     for(int i = 0; i<15; i++){Z_axis(anti_clockwise);}  //up
 }

void __interrupt() Rx_char_USART(void)  // Interrupt function
{     
    if(PIE1bits.RCIE && PIR1bits.RCIF)  // only process USART RX char interrupts
        {
            New_char_RX = true;         // set flag variable
            PIR1bits.RCIF = 0;          // clear this interrupt condition
        }                               // end IF
}                                       // end function


void start_up_menu(void){

    const char Menu[]="1. Default Sequence\n2. Change sequence\n3. Add new component\n4.Remove a component";
    for(int i =0;Menu[i]!='\0';i++)
        serial_tx_char(Menu[i]);
        if(New_char_RX){
            RX_Char = RCREG;
            Switch(RX_Char){
                case '1':  pickandplace_1(); pickandplace_2(); pickandplace_3();//function
                    break;
                case '2':  change_sequence();//change_sequence
                    break;
                case '3':  add_component();//add new component
                    break;
                case '4':  remove_component();//remove a component
                    break;

            }

        }
}