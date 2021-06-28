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
#include "serial_rs232.h"


// Serial & Timing Parameters
static double const Fosc              = 4000000;// Oscillator Frequency in Hz
static double       Tosc              = 1/Fosc; // Tosc in sec
static double const desired_BaudRate  = 9600;    // Desired Baud Rate in bps
static _Bool   New_char_RX = false;
static _Bool component_present = false;
_Bool stop = false; 
int lenth_of_array = 30;
char testarray [30];
char sequence[]="CAB";
unsigned char State='0';
int displacement_X, displacement_Y, rotation;

int X_Pick, X_Place, Y_Pick, Y_Place, Pick_Angle, Place_Angle, X_diff, Y_diff, Angle_diff;
char X_dir, Y_dir, Rot_dir;
unsigned char RX_Char;
volatile unsigned char input_str[10]=" ";
int Steps,Step_X=0, Step_Y=0, Step_Z=0, Step_Angle=0;

///*******************Specific Parameters for each component*********************///

static uint8_t new_TMR1H = 0xFA; 	//new pulse duration (i.e. new position of the servo)
static uint8_t new_TMR1L = 0x60;

#define TMR0H_set   0xB2	// Fine tuned Timer0 registers (PWM period)
#define TMR0L_set   0x04	// 

#define X_Pick_A 3  //Pick and Place Parameters of A
#define Y_Pick_A 4
#define Pick_Angle_A 0

#define X_Place_A 17
#define Y_Place_A 5
#define Place_Angle_A 270    

#define X_Pick_B 3  //Pick and Place Parameters of B
#define Y_Pick_B 10
#define Pick_Angle_B 90 


#define X_Place_B 10
#define Y_Place_B 5     
#define Place_Angle_B 180

#define _XTAL_FREQ 4000000


#define X_Pick_C 3  //Pick and Place Parameters of C
#define Y_Pick_C 15
#define Pick_Angle_C 90 


#define X_Place_C 16
#define Y_Place_C 14     
#define Place_Angle_C 270



int X_Pick_D;   //Pick and Place Parameters of D
int Y_Pick_D; 
int Pick_Angle_D;

int X_Place_D;
int Y_Place_D;
int Place_Angle_D;

int X_diff;
int Y_diff;
int angle_diff;

#define receive_input 1


#define abs(x) ((x) > 0 ? (x) : -(x))
//#define dir(prev,next) ({if(prev<next){return "clockwise";} else{return "anti_clockwise";}) 
#define direct(prev,next) ((prev<next)? 1:0)  //1=clockwise, 0= anticlockwise
/*
 Hardware related definition
 */
 
#define _XTAL_FREQ 200000000 //Crystal Frequency, used in delay
#define speed 1 // Speed Range 10 to 1  10 = lowest , 1 = highest
#define clockwise 1 // clockwise direction macro
#define anti_clockwise 0 // anti clockwise direction macro
#define Open 1
#define Close 0

/*
 *Application related function and definition
 */
 
void system_init (void); // This function will initialise the ports.
void X_axis (char direction);
void Y_axis (char direction);
void Z_axis (char direction);
void Twister (char direction);
void ms_delay(unsigned int val);
int pick_and_place(char Componnt);
int fetch_parameters(char Componnt);
void return_to_initial(void);
void __interrupt() Rx_char_USART (void);
void Z_axis_and_Tweezer(void);
//char direct(int prev,int next);
void start_up_menu(void);
void change_sequence(void);
void add_component(void);
void remove_component(void);
void Tweezer(char action);
void set_new_pos(uint8_t new_TMR1H, uint8_t new_TMR1L);
void check_component(char Compnt);
/*
 * main function starts here
 */
 
void main(void)
{
    //unsigned char RX_Char = ' ';  // used for echo of the received char    unsigned char test_C = ' ';
    init_PORTS();           // PORTS configuration  
    init_USART();           // USART module configuration
    init_interrupts();      // Interrupt configuration (only INT on RX USART enabled)
    
    init_Timers();
    
    ei();                   // enable all interrupts
    T0CONbits.TMR0ON = 1; //Timer 0 enabled, start PWM period count
	T1CONbits.TMR1ON = 1; //Timer 1 enabled, start initial Ton count
	LATCbits.LC2     = 1; //set output on RC2 HIGH (Ton)
    
    while(receive_input){
        New_char_RX=false;
        start_up_menu();        
    }
}


/*System Initialising function to set the pin direction Input or Output*/
 

void start_up_menu(void){
    
    print_string("\n\n1. Start Sequence\n2. Change sequence\n3. Add new component\n4. Remove a component");
    
    while(receive_input){
        if(New_char_RX){
            New_char_RX=false;
            switch(State){
                case '1':
                    print_string("\nStarting sequence:");
                    print_string(sequence);
                    print_string("\nEnter 'Q' to exit to MAIN MENU");
                    return_to_initial();
                        X_diff=0; 
                        Y_diff=0; 
                        Angle_diff=0;
                        //Tweezer(0);
                        while(!stop){
                        for(int i=0; i<strlen(sequence) && !stop;i++){
                            pick_and_place(sequence[i]); 
                        } 
                        }
                        /*
                        do{  
                            
                        }while(!stop || sequence[i++]!=NULL);*/
                    
                    break;
                
                case '2':
                    change_sequence();//change_sequence
                    //return_to_initial();
                    break;
                
                case '3':
                    add_component();//add new component
                    //return_to_initial();
                    break;
                    
                case '4':
                    remove_component();//remove a component
                    //return_to_initial();
                    break;
                    
                case 'Q':
                    New_char_RX=false;
                    stop=false;
                    break;                  
            } 
            break;
        }
    }
}

void change_sequence(void){
    print_string("\nThe current sequence is: ");
    print_string(sequence);
    print_string("\nPlease give the new sequence as input\r");
    int i = 0;
    while(receive_input) {
        if(New_char_RX){
            do{
                sequence[i]=input_str[i];
            }while(input_str[i++]!='\n');
            New_char_RX=false;
            break;
        }
    }
}

void add_component(void){
    char add_componnt[]=" ";
    print_string("\nThe Current sequence is:");
    print_string(sequence);
    print_string("\nEnter the component you want to add: A, B, C or D?");
    while(receive_input){
        if(New_char_RX){
            add_componnt[0]=input_str[0];
            if(add_componnt[0]=='D'){
                New_char_RX=false;
                print_string("\nEnter the pick position X:");
                while(receive_input){
                    if(New_char_RX){
                        X_Pick_D=atoi(input_str);
                        New_char_RX=false;
                        break;
                    }
                }
                print_string("\nEnter the pick position Y:");   
                while(receive_input){
                    if(New_char_RX){
                        Y_Pick_D=(atoi(input_str));
                        New_char_RX=false;
                        break;
                    }
                }
                print_string("\nEnter the pick orientation:");
                while(receive_input){
                    if(New_char_RX){
                        Pick_Angle_D=(atoi(input_str));
                        New_char_RX=false;
                        break;
                    }
                }
                print_string("\nEnter the place position X:");
                while(receive_input){
                    if(New_char_RX){
                        X_Place_D=(atoi(input_str));
                        New_char_RX=false;
                        break;
                    }
                }
                print_string("\nEnter the place position Y:");
                while(receive_input){
                    if(New_char_RX){
                        Y_Place_D=(atoi(input_str));
                        New_char_RX=false;
                        break;
                    }
                }
                print_string("\nEnter the place orientation:");
                while(receive_input){
                        if(New_char_RX){
                        Place_Angle_D=(atoi(input_str));
                        New_char_RX=false;
                        break;
                    }
                }
                break;
            }
            else{
                New_char_RX=false;
                break;
            }
        }
    }
    State='0';
    strncat(sequence,add_componnt,1);
    print_string("\n\nComponent Added: ");
    print_string(add_componnt);
    New_char_RX=false;
}

void remove_component(void){
    print_string("\nWhich component would you like to remove from the  sequence?\n");
    print_string(sequence);
    while(receive_input){
        if(New_char_RX){
            New_char_RX=false;
            char rm_comp = input_str[0];
            char *ptr;  
            int indxToDel;
            ptr=strrchr(sequence,rm_comp);

            if (rm_comp=='Q'){
                break;
            }
            else if(ptr != NULL){
                indxToDel = ptr - sequence;
                memmove(&sequence[indxToDel], &sequence[indxToDel + 1], strlen(sequence) - indxToDel);
                print_string("\ncomponent removed");
                stop=false;
                break;
            }

            else{
                print_string("\ncomponent is not in the sequence. Try selecting another component: \n");
                while(!New_char_RX){}
            }
        }
    }
    New_char_RX=false;
}

/* This method will drive the motor in half-drive mode using direction input */



void X_axis (char direction){  
        if (direction == 0 && !stop){                                                //anticlockwise
            PORTA = 0b00000011;
            ms_delay(speed);
            PORTA = 0b00000110;
            ms_delay(speed);
            PORTA = 0b00001100;
            ms_delay(speed);
            PORTA = 0b00001001;
            ms_delay(speed);
            PORTA = 0b00000011;
            ms_delay(speed);
            Step_X--;
        }
        if (direction == 1 && !stop){                                        //clockwise
            PORTA = 0b00001001;
            ms_delay(speed);
            PORTA = 0b00001100;
            ms_delay(speed);
            PORTA = 0b00000110;
            ms_delay(speed);
            PORTA = 0b00000011;
            ms_delay(speed);
            PORTA = 0b00001001;
            ms_delay(speed);
            Step_X++;
        }
}


void Y_axis (char direction){
        if (direction == 0 && !stop){
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
            Step_Y--;
        }
        if (direction == 1 && !stop){
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
            Step_Y++;
        }   
}

void Z_axis (char direction){
        __delay_ms(20);
        if (direction == 0 && !stop){
            PORTD = 0b00000011;
            ms_delay(10);
            PORTD = 0b00000110;
            ms_delay(10);
            PORTD = 0b00001100;
            ms_delay(10);
            PORTD = 0b00001001;
            ms_delay(10);
            PORTD = 0b00000011;
            ms_delay(10);
            Step_Z--;
        }
        if (direction == 1 && !stop){
            PORTD = 0b00001001;
            ms_delay(10);
            PORTD = 0b00001100;
            ms_delay(10);
            PORTD = 0b00000110;
            ms_delay(10);
            PORTD = 0b00000011;
            ms_delay(10);
            PORTD = 0b00001001;
            ms_delay(10);
            Step_Z++;
        }
}

void Twister (char direction){
        if (direction == 0 && !stop){
            PORTD = 0b00110000;
            ms_delay(speed);
            PORTD = 0b11000000;
            ms_delay(speed);
            PORTD = 0b10010000;
            ms_delay(speed);
            PORTD = 0b00110000;
            ms_delay(speed);
            Step_Angle--;
        }
        if (direction == 1 && !stop){
            PORTD = 0b10010000;
            ms_delay(speed);
            PORTD = 0b01100000;
            ms_delay(speed);
            PORTD = 0b00110000;
            ms_delay(speed);
            PORTD = 0b10010000;
            ms_delay(speed);
            Step_Angle++;
        }
}

void set_new_pos(uint8_t nTMR1H, uint8_t nTMR1L)  // set new position of the servo
{
    TMR1H = nTMR1H;
    TMR1L = nTMR1L;
	
//	SET position = 0° => Ton = 1.5 msec => 1500 Tcyc =>
//	 => overflow after 1500 Tcyc 
//	 => Set initial value of (TMR1H:TMR1L) = 65.536-1.500 = 64.036 = 0xFA24
//	 TMR1H = 0xFA
//	 TMR1L = 0x24
    
//	SET  position at -45° => Ton = 1.25 msec => 1250 Tcyc =>
//	 => overflow after 1250 Tcyc 
//	 => Set initial value of (TMR1H:TMR1L) = 65.536-1.250 = 64.286 = 0xFB1E
//	 TMR1H = 0xFB
//	 TMR1L = 0x1E
	
//	SET  position at +45° => Ton = 1.75 msec => 1750 Tcyc =>
//	 => overflow after 1750 Tcyc 
//	 => Set initial value of (TMR1H:TMR1L) = 65.536-1.750 = 64.036 = 0xF92A
//	 TMR1H = 0xF9
//	 TMR1L = 0x2A
	
    LATCbits.LC2 =  1;       //set output pin RC2 HIGH (Ton)
	T1CONbits.TMR1ON = 1;    //enable Timer 1 => Start of Ton
	   
    return;
}


void Tweezer(char action){
        if(action==1 && !stop){                               //Open       
            new_TMR1H = 0xF9;
            new_TMR1L = 0x53;
            
        }
        if(action == 0 && !stop){                             //close   
            new_TMR1H= 0xFA;
            new_TMR1L= 0x60;
        }
        LATCbits.LC2=1;
        T1CONbits.TMR1ON=1;
}
/*This method will create required delay*/
void ms_delay(unsigned int val)
{
     unsigned int i,j;
        for(i=0;i<val;i++);
        for(j=0;j<1650;j++);                    
}
int fetch_parameters(char Componnt){
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
    
}

void check_component(char Compnt){
    /*int j=0;
    while(!component_present){
        __delay_ms(100);
        print_string("Component missing!");
        print_char(Compnt);
        print_string("missing");
        j++;
        if(j>2){break;}
    }
    component_present=false;*/
    int j = 0;
    while(!stop){
            if(component_present){
                component_present=false;
                break;
            }
            else if(j>1 && !component_present){
                print_string("Component ");
                print_char(Compnt);
                print_string(" missing!");
                component_present=false;
                break;
            }
            else if(!component_present){
                __delay_ms(100);
                j++;}
        }
}
int pick_and_place(char Componnt)
{
    //// Component C //////////////////////////////////
    //Tweezer open 5 units wide
    fetch_parameters(Componnt);
        
    if(!stop){
        print_string("\nPicking component:");
        print_char(Componnt);
    }
        X_dir=direct(X_diff,X_Pick);
        Y_dir=direct(Y_diff, Y_Pick);
        Rot_dir=direct(Angle_diff,Pick_Angle);
        
        for(int i = 0; (i<(abs(X_Pick-X_diff))) && !stop; i++){X_axis(X_dir);}
        
        for(int i = 0; (i<(abs(Y_Pick-Y_diff))) && !stop; i++){Y_axis(Y_dir);}
        
        for(int i = 0; (i<(abs(Angle_diff-Pick_Angle)/3.6)) && !stop; i++){Twister(Rot_dir);}
        Tweezer(Open);
        for(int i=0; i<3 && !stop; i++){Z_axis(clockwise);}
        check_component(Componnt);
        Tweezer(Close);
        for(int i=0; i<3 && !stop; i++){Z_axis(anti_clockwise);}    
        


        X_dir=direct(X_Pick,X_Place);
        Y_dir=direct(Y_Pick,Y_Place);
        Rot_dir=direct(Pick_Angle,Place_Angle);
   if(!stop){
        print_string("\nPlacing component:");
        print_char(Componnt);
    }
        for(int i = 0; (i<(abs(X_Place-X_Pick))) && !stop; i++){X_axis(X_dir);}
        for(int i = 0; (i<(abs(Y_Place-Y_Pick))) && !stop; i++){Y_axis(Y_dir);}
        for(int i = 0; (i<(abs(Pick_Angle-Place_Angle)/3.6)) && !stop; i++){Twister(Rot_dir);}
        for(int i=0; i<3 && !stop; i++){Z_axis(clockwise);}
        Tweezer(Open);
        for(int i=0; i<3 && !stop; i++){Z_axis(anti_clockwise);} 
        Angle_diff= Place_Angle;    
        X_diff = X_Place;
        Y_diff = Y_Place;
       
    
}

void return_to_initial(void){    
    ////Return to Initial position //////////////////////////////////
    New_char_RX=false;
    stop=false;
    Steps=Step_X;
    for(int i = 0; i<Steps; i++){X_axis(anti_clockwise);}
    Steps=Step_Y;
    for(int i = 0; i<Steps; i++){Y_axis(anti_clockwise);}
    Steps=Step_Angle;
    for(int i = 0; i<Steps; i++){Twister(anti_clockwise);}
    Steps=Step_Z;
    for(int i=0; i<Steps && !stop; i++){Z_axis(anti_clockwise);}

}
/*
char direct(int prev, int next){
    if(prev<next){
        return 1;
    }
    else{
        return 0;
    }

}
*/
void Z_axis_and_Tweezer(){
     for(int i = 0;i<15; i++){Z_axis(clockwise);}  //down
     //Tweezer actions, units refer sequence
     for(int i = 0; i<15; i++){Z_axis(anti_clockwise);}  //up
 }

void __interrupt() Rx_char_USART(void)  // Interrupt function
{
    if(PIE1bits.RCIE && PIR1bits.RCIF){
    int i=0;
    do
    {
        while(INTCONbits.INT0IF==0 && !RCIF){};
        input_str[i]=RCREG;
    }while(INTCONbits.INT0IF==0 && input_str[i++] != '\n');
    
    PIR1bits.RCIF = 0;  //clear the interrupt condition
    New_char_RX = true;                               // end IF
    State = input_str[0];
    if(State=="Q"){
        stop=true;
    }
    }
    if(INTCONbits.INT0IF==1 && INTCONbits.INT0IE==1){
        INTCONbits.INT0IF=0;
        //return_to_initial();
        input_str[0]="Q";
        stop=true;
    }
    if(INTCON3bits.INT1IF==1 && INTCON3bits.INT1IE==1){
        INTCON3bits.INT1IF=0;
        component_present=true;
    }
    
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) // process Timer 0 overflow interrupt -> END of Period
        {
            //reset Timer0
	        TMR0H = TMR0H_set;			//set TMR0H
            TMR0L = TMR0L_set;			//set TMR0L
			set_new_pos(new_TMR1H, new_TMR1L);	//set Timer 1
            T0CONbits.TMR0ON = 1;		//Timer 0 enabled (start period)
			T1CONbits.TMR1ON = 1;		//Timer 1 enabled (start Ton)
            INTCONbits.TMR0IF = 0;    	// clear this interrupt condition flag
        }
    if(PIE1bits.TMR1IE && PIR1bits.TMR1IF) // process Timer 1 overflow interrupt -> END of Ton
        {
            LATCbits.LC2 = 0;       //clear output pin (RC2)
            T1CONbits.TMR1ON = 0;   //Stop Timer 1 disabled => END of Ton
	        PIR1bits.TMR1IF = 0;    // clear this interrupt condition flag
        }                           // end IF
}// end function
