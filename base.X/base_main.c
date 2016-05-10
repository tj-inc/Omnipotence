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
#define Start_Threshold 3000 // 12ms * 1000us/ms * 1/4
#define Continue_Threshold 23250 // 93ms * 1000us/ms * 1/4
#define Zero_Threshold 300 // 1.2ms * 1000us/ms * 1/4
#define Void_Threshold 27800 // 110ms * 1000us/ms * 1/4 + 300 to compensate weird behavior

// Function Prototypes
void interrupt interrupt_handler(void);

// Global Variables
// RC Module
char RC_DATA = 0;
char rc_data_index = 0; // Max 7
unsigned int last_CCP1_time;
bit RC_session_activated = 0;
bit RC_session_start_received = 0;
bit RC_session_data_received = 0;
char data_bit_counter = 0;
char data[32];

// System Main
bit mode = 0; // 0 is manual, 1 is auto

void main(void) {
    
    TRISA = 0;
    ANSEL = 0;
    ANSELH = 0;
    
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
    
    // Init CCPR1 and relevant ports
    TRISC1 = 0;
    CCP1M3 = 0; CCP1M2 = 1; CCP1M1 = 0; CCP1M0 = 0; // Capture on every falling edge
	CCP1IE = 1;
	CCP1IF = 0;
    last_CCP1_time = TMR1;
	PEIE = 1;
	GIE = 1;
    
    unsigned int temp_TMR1;
    unsigned int temp_last_CCP1_time;
    unsigned int temp;
    
    while (1) {
        temp_last_CCP1_time = last_CCP1_time;
//        for (char i = 0; i < 100; i++);
        temp_TMR1 = TMR1 + 300;
        temp = temp_TMR1 - temp_last_CCP1_time;
        if ((temp > Void_Threshold)) {
            RC_DATA = 0;
            RC_session_activated = 0;
            RC_session_start_received = 0;
            RC_session_data_received = 0;
            data_bit_counter = 0;
        }
        PORTA = RC_DATA;
    }
}

void interrupt interrupt_handler() {
    if (CCP1IF == 1) {
        if (RC_session_activated == 0) {
            if (RC_session_start_received == 1) {
                unsigned int temp = CCPR1-last_CCP1_time;
                if (temp > Start_Threshold) {
                    RC_session_activated = 1;
                    data_bit_counter = 1;
                }
                RC_session_start_received = 0;
                char i = 0;
            } else {
                RC_session_start_received = 1;
                char i = 0;
            }
        } else {
            if (RC_session_data_received == 0) {
                data_bit_counter++;
                if (data_bit_counter <= 33) {
                    if (CCPR1-last_CCP1_time > Zero_Threshold) {
                        data[data_bit_counter-2] = 1;
                    } else {
                        data[data_bit_counter-2] = 0;
                    }
                } else {
                    RC_session_data_received = 1;
                    RC_session_start_received = 1;
                }
//              if ((data_bit_counter >= 18) & (data_bit_counter <= 25)) { // We can only measure time difference at bit 18 - 25
//                    if (CCPR1 - last_CCP1_time > Zero_Threshold) {
//                        data[data_bit_counter-18] = 1;
//                       RC_DATA++;
//                    } else {
//                        data[data_bit_counter-18] = 0;
//                    }
//                } else if (data_bit_counter > 33) {
//                    RC_session_start_received = 1;
//                }
            } else {
                if (RC_session_start_received == 0) {
                    RC_session_start_received = 1;
                } else {
                    if (CCPR1 - last_CCP1_time > Start_Threshold) {
                        RC_session_activated = 1;
                        RC_session_start_received = 0;
                        RC_session_data_received = 0;
                        data_bit_counter = 1;
                        RC_DATA = 0;
                    } else if (CCPR1 - last_CCP1_time > Continue_Threshold) {
                        RC_session_start_received = 0;
                    }
                }
            }
        }
        last_CCP1_time = CCPR1;
        CCP1IF = 0;
    }
}