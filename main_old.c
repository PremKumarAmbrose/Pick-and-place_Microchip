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
#include <string.h>
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
char sequence[4]="CAB";

int displacement_X, displacement_Y, rotation;

int X_Pick, X_Place, Y_Pick, Y_Place, Pick_Angle, Place_Angle, X_diff=0, Y_diff=0, Angle_diff=0;
char X_dir, Y_dir, Rot_dir;
unsigned char RX_Char;
volatile unsigned char input_str[10]=" ";


///*******************Specific Parameters for each component*********************///

#define X_Pick_A 3  //Pick and Place Parameters of A
#define Y_Pick_A 4
#define Pick_Angle_A 0
/*
char X_Pick_dir_A = clockwise;
char Y_Pick_dir_A = clockwise;
char Rot_Pick_dir_A = clockwise;
*/
#define X_Place_A 17
#define Y_Place_A 5
#define Place_Angle_A 270    
/*
char X_Place_dir_A = clockwise;
char Y_Place_dir_A = clockwise;
char Rot_Place_dir_A = anti_clockwise;
*/
#define X_Pick_B 3  //Pick and Place Parameters of B
#define Y_Pick_B 10
#define Pick_Angle_B 90 

/*
char X_Pick_dir_B = anti_clockwise;
char Y_Pick_dir_B = clockwise;
char Rot_Pick_dir_B = clockwise;
*/
#define X_Place_B 10
#define Y_Place_B 5     
#define Place_Angle_B 180

/*
char X_Place_dir_B = clockwise;
char Y_Place_dir_B = anti_clockwise;
char Rot_Place_dir_B = anti_clockwise;    
*/



#define X_Pick_C 3  //Pick and Place Parameters of C
#define Y_Pick_C 15
#define Pick_Angle_C 90 
/*
 
char X_Pick_dir_C = anti_clockwise;
char Y_Pick_dir_C = clockwise;
char Rot_Pick_dir_C = anti_clockwise;
*/

#define X_Place_C 16
#define Y_Place_C 14     
#define Place_Angle_C 270
/*char X_Place_dir_C = clockwise;
char Y_Place_dir_C = anti_clockwise;
char Rot_Place_dir_C = clockwise;    
*/
int X_Pick_D;   //Pick and Place Parameters of D
int Y_Pick_D; 
int Pick_Angle_D;
/*char X_Pick_dir_D;
char Y_Pick_dir_D;
char Rot_Pick_dir_D;
*/
int X_Place_D;
int Y_Place_D;
int Place_Angle_D;
/*char X_Place_dir_D;
char Y_Place_dir_D;
char Rot_Place_dir_D;    
*/

int X_diff;
int Y_diff;
int angle_diff;


#define abs(x) ((x) > 0 ? (x) : -(x))
//#define dir(prev,next) ({if(prev<next){return "clockwise";} else{return "anti_clockwise";}) 
#define dir(prev,next) ((prev<next)? clockwise: (anti_clockwise)
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
int pick_and_place(char Componnt);
void return_to_initial(void);
void __interrupt() Rx_char_USART (void);
void Z_axis_and_Tweezer(void);
int direct(int prev, int next);
void print_string(char strng[]);
void start_up_menu(void);
void change_sequence(void);
void add_component(void);
void remove_component(void);


/*
 * main function starts here
 */
 
 
void main(void)
{
    unsigned char RX_Char = ' ';  // used for echo of the received char    unsigned char test_C = ' ';
    
    init_PORTS();           // PORTS configuration
    init_USART();           // USART module configuration
    init_interrupts();      // Interrupt configuration (only INT on RX USART enabled)
       
    ei();                   // enable all interrupts
    while(1){
    start_up_menu();        
    }
}


/*System Initialising function to set the pin direction Input or Output*/
 

void start_up_menu(void){

    //const char Menu[]="1. Default Sequence\n2. Change sequence\n3. Add new component\n4. Remove a component";
    
    print_string("\n1. Default Sequence\n2. Change sequence\n3. Add new component\n4. Remove a component");
    /*for(int i =0;Menu[i]!='\0';i++)
        serial_tx_char(Menu[i]);
    */
    while(1){
        if(New_char_RX){

            const char ip = input_str[0];
            //serial_tx_char(RX_Char);
            New_char_RX=false;
            switch(ip){
                case '1':
                while(!New_char_RX){
                    pick_and_place('C');
                    pick_and_place('A');
                    pick_and_place('B');
                    
                    sequence[0]="C";
                    sequence[1]="A";
                    sequence[2]="B";
                }
                
                    break;
                case '2':  change_sequence();//change_sequence
                    break;
                case '3':  add_component();//add new component
                    break;
                case '4':  remove_component();//remove a component
                    break;

            }
            New_char_RX=false;
            break;
        }}
}
void change_sequence(void){
    
    print_string("The current sequence is: ");
    
    
    int j = 0;
    do{
        serial_tx_char(sequence[j]);
        
    }while(sequence[j++] != NULL);
    print_string("Please give the new sequence as input\r");
    
    
    int i = 0;
    while(1) {
        if(New_char_RX){
            print_string("\nThe New sequence is: ");
            do{
                sequence[i]=input_str[i];
                serial_tx_char(sequence[i]);
            }while(input_str[i++]!='\n');
            New_char_RX=false;
            break;
        }
    }
    
    while(!New_char_RX){
        int i =0;
        do{
            pick_and_place(sequence[i]);
        }while(sequence[i++]!='\n');
    }

}

void add_component(void){
    print_string("\nEnter the pick position X:");
    while(1){if(New_char_RX){X_Pick_D=atoi(input_str); New_char_RX=false; break;}}
    print_string("\nEnter the pick position Y:");   
    while(1){if(New_char_RX){Y_Pick_D=(atoi(input_str));New_char_RX=false;break;}}
    print_string("\nEnter the pick orientation:");
    while(1){if(New_char_RX){Pick_Angle_D=(atoi(input_str));New_char_RX=false;break;}}
    print_string("\nEnter the place position X:");
    while(1){if(New_char_RX){X_Place_D=(atoi(input_str));New_char_RX=false;break;}}
    print_string("\nEnter the place position Y:");
    while(1){if(New_char_RX){Y_Place_D=(atoi(input_str));New_char_RX=false;break;}};
    print_string("\nEnter the place orientation:");
    while(1){if(New_char_RX){Place_Angle_D=(atoi(input_str));New_char_RX=false;break;}}
    char add_to_sequence[]="D";
    strncat(sequence, add_to_sequence,1);
    print_string("\nThe current sequence is: ");
    print_string(sequence);
            
    while(!New_char_RX){
        int i =0;        
        do{
            
            pick_and_place(sequence[i]);
            
        }while(sequence[i++]!='\n');
    }
}

void remove_component(void){
    print_string("\nWhich component would you like to remove from the  sequence?\n");
    while(1){
    if(New_char_RX){
        char rm_comp = input_str[0];
        char *ptr;  
        int indxToDel;
        ptr=strrchr(sequence,rm_comp);
        if(ptr != NULL){
            indxToDel = ptr - sequence;
            memmove(&sequence[indxToDel], &sequence[indxToDel + 1], strlen(sequence) - indxToDel);
            break;
        }
        else{
            print_string("\ncomponent is not in the sequence. \nTry selecting another component\n");
            New_char_RX=false;
        } 
    }
    }
    print_string("\nThe new sequence is: ");
    print_string(sequence);
    New_char_RX=false;
    while(!New_char_RX){
        int i =0;
        for(int i=0; i<strlen(sequence);i++){
            
            serial_tx_char(sequence[i]);
            pick_and_place(sequence[i]);
        }
        /*do{
            pick_and_place(sequence[i]);
            serial_tx_char(sequence[i]);
        }while(sequence[i++]!='\\0');*/
    }
}

/* This method will drive the motor in half-drive mode using direction input */

void print_string(char strng[]){
    for(int i =0; strng[i]!='\0'; i++){
        serial_tx_char(strng[i]);
    }
}

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

int pick_and_place(char Componnt)
{
    //// Component C //////////////////////////////////
    //Tweezer open 5 units wide
    if(Componnt=='A')
    {
        X_Pick=X_Pick_A;
        Y_Pick=Y_Pick_A;
        Pick_Angle=Pick_Angle_A;

        X_Place=X_Place_A;
        Y_Place=Y_Place_A;
        Place_Angle=Place_Angle_A;
    }
    else if(Componnt=='B'){
        X_Pick=X_Pick_B;
        Y_Pick=Y_Pick_B;
        Pick_Angle=Pick_Angle_B;

        X_Place=X_Place_B;
        Y_Place=Y_Place_B;
        Place_Angle=Place_Angle_B;
    }
    else if(Componnt=='C'){
        X_Pick=X_Pick_C;
        Y_Pick=Y_Pick_C;
        Pick_Angle=Pick_Angle_C;

        X_Place=X_Place_C;
        Y_Place=Y_Place_C;
        Place_Angle=Place_Angle_C;
    }
    else if(Componnt=='D'){
        X_Pick=X_Pick_D;
        Y_Pick=Y_Pick_D;
        Pick_Angle=Pick_Angle_D;

        X_Place=X_Place_D;
        Y_Place=Y_Place_D;
        Place_Angle=Place_Angle_D;
    }
    else{
        //do nothing
    }
    
    X_dir=direct(X_diff,X_Pick);
    Y_dir=direct(Y_diff, Y_Pick);
    Rot_dir = direct(Angle_diff,Pick_Angle);
    for(int i = 0; i<(abs(X_Pick-X_diff)); i++){X_axis(X_dir);}
    for(int i = 0; i<(abs(Y_Pick-Y_diff)); i++){Y_axis(Y_dir);}
    for(int i = 0; i<((Angle_diff-Pick_Angle)/3.6); i++){Twister(Rot_dir);}
    //ultrasonic check while loop
        Z_axis_and_Tweezer();
        //Tweezer close 4 units wide



    X_dir=direct(X_Pick,X_Place);
    Y_dir=direct(Y_Pick,Y_Place);
    Rot_dir=direct(Pick_Angle,Place_Angle);
    for(int i = 0; i<(abs(X_Place-X_diff)); i++){X_axis(X_dir);}
    for(int i = 0; i<(abs(Y_Place-Y_diff)); i++){Y_axis(Y_dir);}
    for(int i = 0; i<((Pick_Angle-Place_Angle)/3.6); i++){Twister(Rot_dir);}
        Z_axis_and_Tweezer();
        //Tweezer open 5 units wide
    Angle_diff= Place_Angle;    
    X_diff = X_Place;
    Y_diff = Y_Place;
    

}

void return_to_initial(void){    
    ////Return to Initial position //////////////////////////////////
    for(int i = 0; i<X_diff; i++){X_axis(anti_clockwise);}
    for(int i = 0; i<Y_diff; i++){Y_axis(anti_clockwise);}
    for(int i = 0; i<Angle_diff; i++){Twister(anti_clockwise);}

}

int direct(prev,next){
    if(prev<next){
        return clockwise;
    }
    else{
        return anti_clockwise;
    }

}

void Z_axis_and_Tweezer(){
     for(int i = 0;i<15; i++){Z_axis(clockwise);}  //down
     //Tweezer actions, units refer sequence
     for(int i = 0; i<15; i++){Z_axis(anti_clockwise);}  //up
 }

void __interrupt() Rx_char_USART(void)  // Interrupt function
{
    //if(PIE1bits.RCIE&&PIR1bits.RCIF){
    int i=0;
    do{
        
        while(!RCIF){};
        input_str[i]=RCREG;
        //print_string("in while loop");
    }while(input_str[i++]!= '\n');
    //print_string(input_str);
    PIR1bits.RCIF = 0;  //clear the interrupt condition
    New_char_RX = true;                               // end IF
    //serial_tx_char(input_str[0]);
    //serial_tx_char(input_str[1]);
    //serial_tx_char(input_str[2]);
//}                                    
}// end function


