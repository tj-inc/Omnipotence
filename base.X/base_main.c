/*
 * File:   base_main.c
 * Author: Zhou Zbou
 *
 * Created on May 8, 2016, 7:28 PM
 */


#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// Global Defines
enum Buttons {BUTTON_STOP = 0b00000000, BUTTON_OK = 0b00000010, BUTTON_ZERO = 0b01001010, BUTTON_UP = 0b01100010, BUTTON_DOWN = 0b10101000, BUTTON_LEFT = 0b00100010, BUTTON_RIGHT = 0b01001010};
enum RC_States {RC_RESET, RC_START_FALL, RC_START_RISE, RC_RECV_FALL, RC_RECV_RISE, RC_CONT_FALL1, RC_CONT_RISE1, RC_CONT_FALL2, RC_CONT_RISE2};
//#define Start_Threshold 3000 // 12ms * 1000us/ms * 1/4
//#define Continue_Threshold 23250 // 93ms * 1000us/ms * 1/4
//#define Zero_Threshold 300 // 1.2ms * 1000us/ms * 1/4
#define RC_Void_Threshold 27500 // 110ms * 1000us/ms * 1/4
#define RC_Start_Low_Threshold 2200 // 8.8ms * 1000us/ms * 1/4
#define RC_Start_Idle_Threshold 1000 // 4ms * 1000us/ms * 1/4
#define RC_Data_Low_Threshold 1// maybe not need, to be larger than measured
#define RC_Data_Zero_Threshold 375 // 1.5ms * 1000us/ms * 1/4
#define RC_Cont_Idle_Threshold 500 // 2ms * 1000us/ms * 1/4 to be smaller than measured

// Function Prototypes
void interrupt interrupt_handler(void);

// Global Variables
// RC Module
char RC_State = 0;
char RC_index = 0;
char RC_data[8];
unsigned int last_RC_time;
unsigned int last_critical_RC_time;
bit last_RC_data;
bit RC_data_ready = 0;

// System Main
bit mode = 0; // 0 is manual, 1 is auto

void main(void) {
    
    TRISA = 0;
    ANSEL = 0;
    ANSELH = 0;
    
    // Init RB2
    TRISB2 = 1;
    ANS8 = 0;
    nRBPU = 0;
    IOCB2 = 1;
    last_RC_data = RB2;
    RBIF = 0;
    RBIE = 1;
    
    // Init Timer 1
    TMR1GE = 0; TMR1ON = 1; 			//Enable TIMER1 (See Fig. 6-1 TIMER1 Block Diagram in PIC16F887 Data Sheet)
	TMR1CS = 0; 					//Select internal clock whose frequency is Fosc/4, where Fosc = 8 MHz
	T1CKPS1 = 1; T1CKPS0 = 1; 		 	//Set prescale to divide by 4 yielding a clock tick period of 2 microseconds

							/*	From Section 6.12 of PIC16F887 Datasheet:
									bit 5-4 T1CKPS<1:0>: Timer1 Input Clock Prescale Select bits
									11 = 1:8 Prescale Value
									10 = 1:4 Prescale Value
									01 = 1:2 Prescale Value
									00 = 1:1 Prescale Value
							*/
    last_RC_time = TMR1;

	GIE = 1;
    
    unsigned int last_critical_difference;
    unsigned int last_RC_time_backup;
    
    while (1) {
        // RC state transition
        if (((signed int) (TMR1 - last_RC_time)) > RC_Void_Threshold) {
            RC_State = RC_RESET;
            RC_index = 0;
            RC_data_ready = 0;
        } else {
            // Remember to update RC_index
            switch (RC_State) {
                case RC_RESET:
                    if (last_RC_data == 0) {
                        last_critical_RC_time = last_RC_time;
                        RC_State = RC_START_FALL;
                    }
                    break;
                case RC_START_FALL:
                    if (last_RC_data == 1) {
                        last_RC_time_backup = last_RC_time;
                        last_critical_difference = last_RC_time_backup - last_critical_RC_time;
                        if (last_critical_difference > RC_Start_Low_Threshold) {
                            last_critical_RC_time = last_RC_time_backup;
                            RC_State = RC_START_RISE;
                        } else {
                            RC_State = RC_RESET;
                        }
                    }
                    break;
                case RC_START_RISE:
                    if (last_RC_data == 0) {
                        last_RC_time_backup = last_RC_time;
                        last_critical_difference = last_RC_time_backup - last_critical_RC_time;
                        if (last_critical_difference > RC_Start_Idle_Threshold) {
                            last_critical_RC_time = last_RC_time_backup;
                            RC_State = RC_RECV_FALL;
                        } else {
                            RC_State = RC_RESET;
                        }
                    }
                    break;
                case RC_RECV_FALL:
                    if (last_RC_data == 1) {
                        RC_State = RC_RECV_RISE;
                    }
                    break;
                case RC_RECV_RISE:
                    if (last_RC_data == 0) {
                        last_RC_time_backup = last_RC_time;
                        if (RC_index == 32) {
                            RC_State = RC_CONT_FALL1;
                            RC_data_ready = 1;
                        } else {
                            last_critical_difference = last_RC_time_backup - last_critical_RC_time;
                            if ((16 <= RC_index) & (RC_index <= 23)) {
                                if (last_critical_difference < RC_Data_Zero_Threshold) {
                                    RC_data[RC_index-16] = 0;
                                } else {
                                    RC_data[RC_index-16] = 1;
                                }
                            }
                            last_critical_RC_time = last_RC_time_backup;
                            RC_index++;
                            RC_State = RC_RECV_FALL;
                            RC_data_ready = 0;
                        }
                    }
                    break;
                case RC_CONT_FALL1:
                    if (last_RC_data == 1) {
                        last_RC_time_backup = last_RC_time;
                        last_critical_difference = last_RC_time_backup - last_critical_RC_time;
                        if (last_critical_difference > RC_Start_Low_Threshold) {
                            last_critical_RC_time = last_RC_time_backup;
                            RC_State = RC_CONT_RISE1;
                        } else {
                            RC_State = RC_CONT_RISE2;
                        }
                    }
                    break;
                case RC_CONT_RISE1:
                    if (last_RC_data == 0) {
                        last_RC_time_backup = last_RC_time;
                        last_critical_difference = last_RC_time_backup - last_critical_RC_time;
                        if (last_critical_difference > RC_Start_Idle_Threshold) {
                            // Starting a new session
                            last_critical_RC_time = last_RC_time_backup;
                            RC_State = RC_RECV_FALL;
                            RC_index = 0;
                        } else if (last_critical_difference > RC_Cont_Idle_Threshold) {
                            // Keep current session
                            last_critical_RC_time = last_RC_time_backup;
                            RC_State = RC_CONT_FALL2;
                        } // Another else can be added for debugging purposes
                    }
                    break;
                case RC_CONT_FALL2:
                    if (last_RC_data == 1) {
                        RC_State = RC_CONT_RISE2; // Won't work if there is interference
                    }
                    break;
                case RC_CONT_RISE2:
                    if (last_RC_data == 0) {
                        last_critical_RC_time = last_RC_time;
                        RC_State = RC_CONT_FALL1; // Won't work if there is interference
                    }
            }
        }
    }
}

void interrupt interrupt_handler() {
    if (RBIF == 1) {
        last_RC_time = TMR1;
        last_RC_data = RB2;
        RBIF = 0;
    }
}