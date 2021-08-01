/*

 * * File:   main.c
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
#include "serial.h"


// Serial & Timing Parameters
static double const Fosc              = 4000000;                    // Oscillator Frequency in Hz
static double       Tosc              = 1/Fosc;                     // Tosc in sec
static double const desired_BaudRate  = 9600;                       // Desired Baud Rate in bps
#define _XTAL_FREQ  4000000


static uint8_t new_TMR1H = 0xFA;                                    //new pulse duration (i.e. new position of the servo)
static uint8_t new_TMR1L = 0x60;
#define TMR0H_set   0xB2                                            // Fine tuned Timer0 registers (PWM period)
#define TMR0L_set   0x04	


#define receive_input 1                                             //used for while(true) loop
static _Bool   New_char_RX = false;                                 //used for the reception of new char as input
static _Bool   component_present = false;                           //used to check if the component is present in the pick position    
static _Bool   stop = false;                                        //used to stop the loop and exit to main menu


volatile unsigned char input_str[10]=" ";                           //used to collect the input as string and store as an array of characters
char                   sequence[]="CAB";                                              //default loop sequence
unsigned char          func='0';                                    //used for switching the cases

int  X_Pick, X_Place, Y_Pick, Y_Place, Pick_Angle, Place_Angle;     //used as temporary variables for each component every time of pick and place
int  X_diff, Y_diff,  Angle_diff;                                    //variable to keep the difference in the position between current component in loop and the next component
char X_dir,  Y_dir,   Twist_dir;                                       //temporary variable to assign the direction of rotation iteratively for each component's pick and place 

                                                                                
int  Steps, Step_X=0, Step_Y=0, Step_Z=0, Step_Angle=0;             //steps counter to keep track of the position of the stepper motors    

int  steps_per_unit=3;                                              //to define the number of steps to be moved for a unit value of movement



///*******************Specific values of position and orientation angle for each component*********************///
#define X_Pick_A 3  //Pick and Place values of A
#define Y_Pick_A 4
#define Pick_Angle_A 0

#define X_Place_A 17
#define Y_Place_A 5
#define Place_Angle_A 270    

#define X_Pick_B 3  //Pick and Place values of B
#define Y_Pick_B 10
#define Pick_Angle_B 90 


#define X_Place_B 10
#define Y_Place_B 5     
#define Place_Angle_B 180




#define X_Pick_C 3  //Pick and Place values of C
#define Y_Pick_C 15
#define Pick_Angle_C 90 


#define X_Place_C 16
#define Y_Place_C 14     
#define Place_Angle_C 270


int X_Pick_D;   //Pick and Place values of D. Will be assigned by user
int Y_Pick_D; 
int Pick_Angle_D;

int X_Place_D;
int Y_Place_D;
int Place_Angle_D;

int X_diff;
int Y_diff;
int angle_diff;



#define speed 1                                     // Speed Range 10 to 1  10 = lowest , 1 = highest
#define abs(x) ((x) > 0 ? (x) : -(x))               //macro to give the absolute value of x
#define direct(prev,next) ((prev<next)? 1:0)        //macro to calculate the direction of motors. 1=clockwise, 0= anticlockwise


/*
 Hardware related definition
 */
 
//#define _XTAL_FREQ 200000000 //Crystal Frequency, used in delay


#define clockwise 1                                 // clockwise direction macro
#define anti_clockwise 0                            // anti clockwise direction macro
#define Open 1
#define Close 0


void system_init (void);                           // This function will initialise the ports.
void ms_delay(unsigned int val);                   //This is used to cause delay in the stepper motors
int pick_and_place(char Componnt);                //This fucntion is used for main pick and place loop sequence
int fetch_coordinates(char Componnt);             //This function fetches the coordinates and other values required for the sequence
void return_to_initial(void);                     //This function is used to return to the initial position
void __interrupt() Rx_char_USART (void);          //Interrupt function
void start_up_menu(void);                         //Provides a menu to switch between different functions
void change_sequence(void);                       //This function is used to change the order of loop sequence
void add_component(void);                         //This function is used to add a new component to the loop sequence
void remove_component(void);                      //This function is used to remove an existing component from the loop sequence
void set_new_pos(uint8_t new_TMR1H, uint8_t new_TMR1L); //Function used to set the timers
void check_component(char Compnt);                //This function checks if the desired component is present in the pick position 
void Tweezer(char action);                         //Function to open and close tweezer
void X_axis (char direction);                      //Function to move the stepper motor in X axis
void Y_axis (char direction);                      //Function to move the stepper motor in Y axis
void Z_axis (char direction);                      //Function to move the stepper motor in Z axis
void Twister (char direction);                     //Function used to rotate the twister for orientation


/*
 * main function starts here
 */
 
void main(void)
{
    init_PORTS();                   // PORTS configuration  
    init_USART();                   // USART module configuration
    init_interrupts();              // Interrupt configuration (only INT on RX USART enabled)
    init_Timers();                  // Timers Configuration
    
    ei();                           // enable all interrupts
    T0CONbits.TMR0ON = 1;           //Timer 0 enabled, start PWM period count
	T1CONbits.TMR1ON = 1;           //Timer 1 enabled, start initial Ton count
	LATCbits.LC2     = 1;           //set output on RC2 HIGH (Ton)
    
    while(receive_input){
        New_char_RX=false;
        start_up_menu();            //Main Menu of the system
    }
}

///This is the main menu of the system, this is called everytime the system is interrupted by an emergency stop button//
///This is where all the switching of several user desired funtion happen///
void start_up_menu(void){
    
    print_string("\n\n1. Start Sequence\n2. Change sequence\n3. Add new component\n4. Remove a component\n5. Return to Initial Position");
    
    while(receive_input){
        if(New_char_RX){
            New_char_RX=false;
            func = input_str[0];                    //reading the switch case from the input
            switch(func){                           //switched between different user desired functions to alter the sequence
                
                case '1':                           //This Case is when the loop sequence is initiated
                    print_string("\n\nStarting sequence:");
                    print_string(sequence); 
                    return_to_initial();
                    X_diff=0;                       //This is used to keep track of the position of X (difference between current and previous X positions)   
                    Y_diff=0;                       //This is used to keep track of the position of Y (difference between current and previous X positions)
                    Angle_diff=0;                   //This is used to keep track of the angle of rotation(difference between current and previous Angle of rotation)
          
                    while(!stop){                   //while the loop is not stoppe by external interrupt, It keeps running
                        for(int i=0; i<strlen(sequence) && !stop;i++){
                            pick_and_place(sequence[i]); 
                        } 
                    }
                    break;
                
                case '2':
                    change_sequence();              //call for function to change the loop sequence
                    break;
                
                case '3':
                    add_component();                //call for function to add new component
                    break;
                    
                case '4':
                    remove_component();             //call to remove a component
                    break;

                case '5':
                    print_string("\n\nReturning to initial position..");
                    return_to_initial();            //to return to initial position
                    break;
            } 
            break;
        }
    }
}



//////////*This function changes the order of components in the loop sequence*///////////
void change_sequence(void){
    print_string("\nThe current sequence is: ");
    print_string(sequence);
    print_string("\nPlease give the new sequence as input\r");
    int i = 0;
    while(receive_input) {
        if(New_char_RX){
            do{
                sequence[i]=input_str[i];               //input string is added to the sequence iteratively
            }while(input_str[i++]!='\n');
            
            New_char_RX=false;
            break;
        }
    }
}



////////////*Function used to add the desired component to the loop sequence*//////////////
void add_component(void){
    char add_componnt[]=" ";
    print_string("\n\nThe Current sequence is:");
    print_string(sequence);
    print_string("\nEnter the component you want to add: A, B, C or D?");
    while(receive_input){
        if(New_char_RX && input_str[0]=='D'){           //If the name of the new component added is 'D', then the user inputs the coordinates and other parameters
            add_componnt[0]=input_str[0];               //The input char is assigned to a variable to use it for altering the loop sequence
            New_char_RX=false;
            print_string("\nEnter the pick position X (1-20): ");
            while(receive_input){
                if(New_char_RX){
                    X_Pick_D=atoi(input_str);           //pick position X is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            print_string("\nEnter the pick position Y (1-20): ");   
            while(receive_input){
                if(New_char_RX){
                    Y_Pick_D=(atoi(input_str));         //pick position Y is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            print_string("\nEnter the pick orientation (0-360): ");
            while(receive_input){
                if(New_char_RX){
                    Pick_Angle_D=(atoi(input_str));     //pick orientation is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            print_string("\nEnter the place position X (1-20): ");
            while(receive_input){
                if(New_char_RX){
                    X_Place_D=(atoi(input_str));        //place position X is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            print_string("\nEnter the place position Y (1-20): ");
            while(receive_input){
                if(New_char_RX){
                    Y_Place_D=(atoi(input_str));        //place position Y is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            print_string("\nEnter the place orientation (0-360): ");
            while(receive_input){
                    if(New_char_RX){
                    Place_Angle_D=(atoi(input_str));    //place orientation is received as input and converted into integer
                    print_string(input_str);
                    New_char_RX=false;
                    break;
                }
            }
            break;
        }
        else if(New_char_RX){                           //Else if it's not 'D' the component A, B, or C can be added to the sequence with the default values already defined
            add_componnt[0]=input_str[0];               //The input char is assigned to a variable to use it for altering the loop sequence    
            New_char_RX=false;
            break;
        }
        
    }
    strcat(sequence,add_componnt);                      //The newly added component is added to the sequence variable at the end of the existing sequence
    print_string("\n\nComponent Added: ");     
    print_string(add_componnt);
    New_char_RX=false;
}



////////////*Function used to remove the desired component from the loop sequence*//////////////
void remove_component(void){
    print_string("\nWhich component would you like to remove from the  sequence?\n");
    print_string(sequence);
    while(receive_input){
        if(New_char_RX){
            New_char_RX=false;
            char rm_comp = input_str[0];                //The component to be removed is received as input and assigned to the variable
            char *ptr;  
            int indxToDel;
            ptr=strrchr(sequence,rm_comp);              //returns the pointer of the last occurrence of the desired component to be removed from the loop sequence
            
            if(ptr != NULL){
                indxToDel = ptr - sequence;             //index of the component to be removed from the sequence is assigned
                memmove(&sequence[indxToDel], &sequence[indxToDel + 1], strlen(sequence) - indxToDel);  //the last occurrence of the component is removed from the sequence with the 
                print_string("\ncomponent removed");
                stop=false;
                break;
            }
            
            /*else condition is run if the component supposed to be deleted is not found in the existing sequence*/
            else{
                print_string("\ncomponent is not in the sequence. Try selecting another component: \n");
                while(!New_char_RX){}
            }
        }
    }
    New_char_RX=false;
}



//////////* This function will drive the stepper motor in X-axis in wave-drive mode *///////////
void X_axis (char direction){  
        if (direction == 0){         //anticlockwise
            PORTA = 0b00000001;
            ms_delay(speed);
            PORTA = 0b00000011;
            ms_delay(speed);
            PORTA = 0b00000010;
            ms_delay(speed);
            PORTA = 0b00000110;
            ms_delay(speed);
            PORTA = 0b00000100;
            ms_delay(speed);
            PORTA = 0b00001100;
            ms_delay(speed);
            PORTA = 0b00001000;
            ms_delay(speed);
            PORTA = 0b00001001;
            ms_delay(speed);
            Step_X--;               //Used to keep track of the position of stepper motor to return to the initial position
        }
        if (direction == 1){         //clockwise
            PORTA = 0b00001001;
            ms_delay(speed);
            PORTA = 0b00001000;
            ms_delay(speed);
            PORTA = 0b00001100;
            ms_delay(speed); 
            PORTA = 0b00000100;
            ms_delay(speed);
            PORTA = 0b00000110;
            ms_delay(speed);
            PORTA = 0b00000010;
            ms_delay(speed);
            PORTA = 0b00000011;
            ms_delay(speed);
            PORTA = 0b00000001;
            ms_delay(speed);
            Step_X++;               //Used to keep track of the position of stepper motor to return to the initial position
        }
}


///////////* This function will drive the stepper motor in Y-axis in half-drive mode *///////////
void Y_axis (char direction){
        if (direction == 0){        //anticlockwise
            PORTB = 0b00010000;
            ms_delay(speed);
            PORTB = 0b00110000;
            ms_delay(speed);
            PORTB = 0b00100000;
            ms_delay(speed);
            PORTB = 0b01100000;
            ms_delay(speed);
            PORTB = 0b01000000;
            ms_delay(speed);
            PORTB = 0b11000000;
            ms_delay(speed);
            PORTB = 0b10000000;
            ms_delay(speed);
            PORTB = 0b10010000;
            ms_delay(speed);
            Step_Y--;               //Used to keep track of the position of stepper motor to return to the initial position
        }
        if (direction == 1){        //clockwise
            PORTB = 0b10010000;
            ms_delay(speed);
            PORTB = 0b10000000;
            ms_delay(speed);
            PORTB = 0b11000000;
            ms_delay(speed); 
            PORTB = 0b01000000;
            ms_delay(speed);
            PORTB = 0b01100000;
            ms_delay(speed);
            PORTB = 0b00100000;
            ms_delay(speed);
            PORTB = 0b00110000;
            ms_delay(speed);
            PORTB = 0b00010000;
            ms_delay(speed);
            Step_Y++;               //Used to keep track of the position of stepper motor to return to the initial position
        }   
}

////////////* This function will drive the stepper motor in Z-axis in half-drive mode *////////////
void Z_axis (char direction){
        if (direction == 0){        //anticlockwise
            PORTD = 0b00010000;
            ms_delay(speed);
            PORTD = 0b00110000;
            ms_delay(speed);
            PORTD = 0b00100000;
            ms_delay(speed);
            PORTD = 0b01100000;
            ms_delay(speed);
            PORTD = 0b01000000;
            ms_delay(speed);
            PORTD = 0b11000000;
            ms_delay(speed);
            PORTD = 0b10000000;
            ms_delay(speed);
            PORTD = 0b10010000;
            ms_delay(speed);
            Step_Z--;           //Used to keep track of the position of stepper motor to return to the initial position
        }
        if (direction == 1){        //clockwise
            PORTD = 0b10010000;
            ms_delay(speed);
            PORTD = 0b10000000;
            ms_delay(speed);
            PORTD = 0b11000000;
            ms_delay(speed); 
            PORTD = 0b01000000;
            ms_delay(speed);
            PORTD = 0b01100000;
            ms_delay(speed);
            PORTD = 0b00100000;
            ms_delay(speed);
            PORTD = 0b00110000;
            ms_delay(speed);
            PORTD = 0b00010000;
            ms_delay(speed);
            Step_Z++;           //Used to keep track of the position of stepper motor to return to the initial position
        }
}

/////////////* This function will drive the stepper motor for the orientation in half-drive mode *////////////
void Twister (char direction){
        if (direction == 0){        //anticlockwise
            PORTD = 0b00000001;
            ms_delay(speed);
            PORTD = 0b00000011;
            ms_delay(speed);
            PORTD = 0b00000010;
            ms_delay(speed);
            PORTD = 0b00000110;
            ms_delay(speed);
            PORTD = 0b00000100;
            ms_delay(speed);
            PORTD = 0b00001100;
            ms_delay(speed);
            PORTD = 0b00001000;
            ms_delay(speed);
            PORTD = 0b00001001;
            ms_delay(speed);
            Step_Angle--;               //Used to keep track of the position of stepper motor to return to the initial position
        }
        if (direction == 1){        //clockwise
            PORTD = 0b00001001;
            ms_delay(speed);
            PORTD = 0b00001000;
            ms_delay(speed);
            PORTD = 0b00001100;
            ms_delay(speed); 
            PORTD = 0b00000100;
            ms_delay(speed);
            PORTD = 0b00000110;
            ms_delay(speed);
            PORTD = 0b00000010;
            ms_delay(speed);
            PORTD = 0b00000011;
            ms_delay(speed);
            PORTD = 0b00000001;
            ms_delay(speed);
            Step_Angle++;               //Used to keep track of the position of stepper motor to return to the initial position
        }
}


///////*This method will create required delay by taking 1 to 10 as input for 10 levels of the delay*/////////
void ms_delay(unsigned int val)
{
    unsigned int i,j;
    for(i=0;i<val;i++);
    for(j=0;j<1650;j++);                    
}


void set_new_pos(uint8_t nTMR1H, uint8_t nTMR1L)  // set new position of the servo
{
    TMR1H = nTMR1H;
    TMR1L = nTMR1L;
    LATCbits.LC2 =  1;               //set output pin RC2 HIGH (Ton)
	T1CONbits.TMR1ON = 1;            //enable Timer 1 => Start of Ton
	   
    return;
}
void Tweezer(char action){
    if(action == 1 && !stop){          //Open       
        new_TMR1H = 0xF9;            //set to open 45 
        new_TMR1L = 0x53;

    }
    if(action == 0 && !stop){        //close   
        new_TMR1H= 0xFA;             //set to close back to 0 degree as initial position
        new_TMR1L= 0x60;
    }
    LATCbits.LC2=1;                  //set output pin RC2 HIGH (Ton)
    T1CONbits.TMR1ON=1;              //enable Timer 1 => Start of Ton
}


///*******This function assigns the coordinates to be used to run the motors every time for each component******///
/*****This function is called in the pick and place loop to fetch the positions*****/
int fetch_coordinates(char Componnt){
    if(Componnt=='A'){               //For the component A
        X_Pick=X_Pick_A;            
        Y_Pick=Y_Pick_A;
        Pick_Angle=Pick_Angle_A;

        X_Place=X_Place_A;
        Y_Place=Y_Place_A;
        Place_Angle=Place_Angle_A;
    }
    else if(Componnt=='B'){         //For the component B
        X_Pick=X_Pick_B;
        Y_Pick=Y_Pick_B;
        Pick_Angle=Pick_Angle_B;

        X_Place=X_Place_B;
        Y_Place=Y_Place_B;
        Place_Angle=Place_Angle_B;
    }
    else if(Componnt=='C'){         //For the component C
        X_Pick=X_Pick_C;
        Y_Pick=Y_Pick_C;
        Pick_Angle=Pick_Angle_C;

        X_Place=X_Place_C;
        Y_Place=Y_Place_C;
        Place_Angle=Place_Angle_C;
    }
    else if(Componnt=='D'){         //For the component D
        X_Pick=X_Pick_D;
        Y_Pick=Y_Pick_D;
        Pick_Angle=Pick_Angle_D;

        X_Place=X_Place_D;
        Y_Place=Y_Place_D;
        Place_Angle=Place_Angle_D;
    }
    
}


/****Function used to check and verify the presence of component in the pick and position and alert if it is not present****/
void check_component(char Compnt){
    int j = 0;
    while(!stop){
        if(component_present){                      //if the component is available, system simply proceeds with the picking of component
            component_present=false;         
            break;
        }
        else if(j>0 && !component_present){         //if the component is not present after few seconds, system sends an alert and stops the sequence
            print_string("\nComponent ");
            print_char(Compnt);
            print_string(" missing!");
            stop=true;                              //sets stop=true to stop the sequence and get response from the user
            break;
        }
        else if(!component_present){
            print_string("\nwaiting for few seconds for the component..");
            __delay_ms(5000);                        //delay makes the system to wait for few seconds 
            j++;
        }
    }
}


int pick_and_place(char Componnt)
{
    fetch_coordinates(Componnt);
        
    print_string("\nPicking component: "); 
    print_char(Componnt);
    
    
    /* direction of rotation for the stepper motors are calculated by using the macro(direct) defined in the beginning */
    /* the macro(direct) compares the previous value with the current value and gives 1 (clockwise) */
    /* if the current value is higher than previous value else 0 (anti-clockwise) */
    X_dir=direct(X_diff,X_Pick);            
    Y_dir=direct(Y_diff, Y_Pick);
    Twist_dir=direct(Angle_diff,Pick_Angle);
    
    
    
    ///loop for each stepper motor for the number of steps ///
    ///the number of steps is calculated by finding the absolute differnece between the previous place position and current pick position///
    ///steps_per_unit is used to change the number of steps the motor moves for a single unit///
    ///for the twister the angle is divided by '3.6' which gives 25 steps for a 90 degree rotation, 50 steps for a 180 degree, 100 steps for 270 degree  and so on..///
    
    for(int i = 0; (i<((abs(X_Pick-X_diff))*steps_per_unit)) && !stop; i++){
        X_axis(X_dir);   
    }
    for(int i = 0; (i<((abs(Y_Pick-Y_diff))*steps_per_unit)) && !stop; i++){
        Y_axis(Y_dir);
    }
    for(int i = 0; (i<(abs(Angle_diff-Pick_Angle)/3.6)) && !stop; i++){
        Twister(Twist_dir);
    }
    
    Tweezer(Open);                                                              //Tweezer is opened
    for(int i=0; i<(3*steps_per_unit) && !stop; i++){Z_axis(clockwise);}        //the hand is moved down three units
    check_component(Componnt);                                                  //checks if the component is present in the pick position
    
    Tweezer(Close);                                                             //Tweezer is closed
    for(int i=0; i<(3*steps_per_unit) && !stop; i++){Z_axis(anti_clockwise);}   //The hand is moved up three units       
    

    /*direction of rotation is calculated again to place the component*/
    X_dir=direct(X_Pick,X_Place);
    Y_dir=direct(Y_Pick,Y_Place);
    Twist_dir=direct(Pick_Angle,Place_Angle);
    
    
    if(!stop){
        print_string("\nPlacing component: ");
        print_char(Componnt);
    }
    
    ////same repeats for the placing of the component////
    ///here the steps is calculated as the absolute difference between the pick position and place position///
    for(int i = 0; (i<((abs(X_Place-X_Pick))*steps_per_unit)) && !stop; i++){
        X_axis(X_dir);
    }
    for(int i = 0; (i<((abs(Y_Place-Y_Pick))*steps_per_unit)) && !stop; i++){
        Y_axis(Y_dir);
    }
    for(int i = 0; (i<(abs(Pick_Angle-Place_Angle)/3.6)) && !stop; i++){
        Twister(Twist_dir);
    }
    
        
    for(int i=0; i<(3*steps_per_unit) && !stop; i++){Z_axis(clockwise);}        //the hand is moved down three units
    Tweezer(Open);                                                              //Tweezer is opened
    for(int i=0; i<(3*steps_per_unit) && !stop; i++){Z_axis(anti_clockwise);}   //the hand is moved up three units
    
    ///sets the difference values to be used for the next component///
    Angle_diff= Place_Angle;
    X_diff = X_Place;
    Y_diff = Y_Place; 
}




////Function used to return to Initial position ////////////////////////////////
void return_to_initial(void){   
    New_char_RX=false;
    stop=false;
    
    Steps=Step_X;
    for(int i = 0; i<Steps; i++){X_axis(anti_clockwise);}
    
    Steps=Step_Y;
    for(int i = 0; i<Steps; i++){Y_axis(anti_clockwise);}
    
    Steps=Step_Angle;
    for(int i = 0; i<Steps; i++){Twister(anti_clockwise);}
    
    Steps=Step_Z;
    for(int i=0; i<Steps; i++){Z_axis(anti_clockwise);}    
    
    Tweezer(Close);
}


/// Interrupt function ///
void __interrupt() Rx_char_USART(void)  
{
    if(PIE1bits.RCIE && PIR1bits.RCIF){
        int i=0;
        do
        {
            while(INTCONbits.INT0IF==0 && !RCIF){};
            input_str[i]=RCREG;                                                 //input is stored as an array of characters in this variable iteratively
        }while(INTCONbits.INT0IF==0 && input_str[i++] != '\n');                 //loop continues until '\n' newline indicator

        PIR1bits.RCIF = 0;                                                      //clear the interrupt condition
        New_char_RX = true;                              
        
    }
    
    
    
    if(INTCONbits.INT0IF && INTCONbits.INT0IE){                                 //interrupt when the emergency stop button is pressed, goes to the main menu
        INTCONbits.INT0IF=0;                                                    // clear this interrupt condition flag
        stop=true;
    }
    
    
    
    if(INTCON3bits.INT1IF && INTCON3bits.INT1IE){                               //interrupt when the proximity interrupts when the component is present in the pick position
        INTCON3bits.INT1IF=0;                                                   // clear this interrupt condition flag
        component_present=true;
    }
    
    
    
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)                                  // process Timer 0 overflow interrupt -> END of Period
        {
            //reset Timer0
	        TMR0H = TMR0H_set;                                                  //set TMR0H
            TMR0L = TMR0L_set;                                                  //set TMR0L
			set_new_pos(new_TMR1H, new_TMR1L);                                  //set Timer 1
            T0CONbits.TMR0ON = 1;                                               //Timer 0 enabled (start period)
			T1CONbits.TMR1ON = 1;                                               //Timer 1 enabled (start Ton)
            INTCONbits.TMR0IF = 0;                                              // clear this interrupt condition flag
        }
    
    
    if(PIE1bits.TMR1IE && PIR1bits.TMR1IF)                                      // process Timer 1 overflow interrupt -> END of Ton
        {
            LATCbits.LC2 = 0;                                                   //clear output pin (RC2)
            T1CONbits.TMR1ON = 0;                                               //Stop Timer 1 disabled => END of Ton
	        PIR1bits.TMR1IF = 0;                                                // clear this interrupt condition flag
        }                                                       
}
