/*
 * File:   base_main.c
 * Author: Zhou Zbou, Henry Teng
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
// RC Modules
enum Buttons {BUTTON_STOP, BUTTON_UP, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_DOWN, BUTTON_OK, BUTTON_ZERO};
enum RC_States {RC_RESET, RC_START_FALL, RC_START_RISE, RC_RECV_FALL, RC_RECV_RISE, RC_CONT_FALL1, RC_CONT_RISE1, RC_CONT_FALL2, RC_CONT_RISE2};
#define RC_Void_Threshold 27500 // 110ms * 1000us/ms * 1/4
#define RC_Start_Low_Threshold 2200 // 8.8ms * 1000us/ms * 1/4
#define RC_Start_Idle_Threshold 1000 // 4ms * 1000us/ms * 1/4
#define RC_Data_Low_Threshold 1// maybe not need, to be larger than measured
#define RC_Data_Zero_Threshold 375 // 1.5ms * 1000us/ms * 1/4
#define RC_Cont_Idle_Threshold 500 // 2ms * 1000us/ms * 1/4 to be smaller than measured

// MC Module
enum Motions {CMD_STOP, CMD_FORWARD, CMD_LEFT, CMD_RIGHT, CMD_BACKWARD};
enum MC_Command_Outputs {STOP = 0, FORWARD = 0b0101, BACKWARD = 0b1010, TURN_LEFT = 0b0110, TURN_RIGHT = 0b1001};
#define MC_HIGH_PULSE 37800 // 90% duty cycle
#define MC_LOW_PULSE  4200
#define ENA RB0
#define ENB RB1
#define MC_TOP_COMMAND_OUT PORTD
#define MC_BASE_COMMAND_OUT PORTA

// Mode
#define mode RC4

// Trigger
#define pull_trigger RC5

// Function Prototypes
char RC_return_key(void);
void MC_set_motion(char);
void interrupt interrupt_handler(void);

// Global Variables
// RC Module
char RC_State = 0;
char RC_index = 0;
char RC_data[3];
unsigned int last_RC_time;
unsigned int last_critical_RC_time;
bit last_RC_data;
bit RC_data_ready = 0;

// MC Module
char last_motion = CMD_STOP;

void main(void) {
    // Init RC4 and RC5 for mode and trigger
    TRISC = 0;
    PORTC = 0;
    
    // Init RD1:0 for top's motion control signal
    TRISD = 0x03;
    
    // Init RB2 for RC Module, RB1 and RB0, RA3:0 for MC Module
    TRISB = 0b100;
    TRISA = 0;
    ANSEL = 0;
    ANSELH = 0;
    nRBPU = 0;
    IOCB2 = 1;
    last_RC_data = RB2;
    RBIF = 0;
    RBIE = 1;
    ENA = 0;
    ENB = 0;
    
    // Init Timer 1
    TMR1GE = 0; TMR1ON = 1; 			//Enable TIMER1 (See Fig. 6-1 TIMER1 Block Diagram in PIC16F887 Data Sheet)
	TMR1CS = 0; 					//Select internal clock whose frequency is Fosc/4, where Fosc = 8 MHz
	T1CKPS1 = 1; T1CKPS0 = 1; 		 	//Set prescale to divide by 4 yielding a clock tick period of 2 microseconds
    last_RC_time = TMR1;
    
    // Init CCPR1
    CCP1M3 = 1;
    CCP1M2 = 0;
    CCP1M1 = 1;
    CCP1M0 = 0;
    CCP1IF = 0;
    CCP1IE = 1;
    CCPR1 = TMR1 + 10;
    
    // Turn on Interrupts
    PEIE = 1;
	GIE = 1;
    
    unsigned int last_critical_difference;
    unsigned int last_RC_time_backup;
    char RC_key = BUTTON_STOP;
    char last_RC_key; // This is debouncing for mode switching (one press yields one switch)
    
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
                            if ((16 <= RC_index) & (RC_index <= 18)) { // We only need first 3 bits
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
        last_RC_key = RC_key;
        RC_key = RC_return_key();
        // Update mode
        if ((RC_key == BUTTON_ZERO) & (last_RC_key != BUTTON_ZERO)) {
            mode = ~mode;
        }
        
        // Update pull_trigger
        pull_trigger = (RC_key == BUTTON_OK);
        
        if (mode) {
            // auto
            MC_set_motion(MC_TOP_COMMAND_OUT);
        } else {
            // manual
            MC_set_motion(((RC_key == BUTTON_ZERO) | RC_key == BUTTON_OK) ? BUTTON_STOP : RC_key);
        }
    }
}

char RC_return_key() {
    if (RC_data_ready) {
        if (RC_data[0]) {
            if (RC_data[1]) {
                return BUTTON_RIGHT;
            } else {
                return BUTTON_DOWN;
            }
        } else {
            if (RC_data[1]) {
                if (RC_data[2]) {
                    return BUTTON_UP;
                } else {
                    return BUTTON_ZERO;
                }
            } else {
                if (RC_data[2]) {
                    return BUTTON_LEFT;
                } else {
                    return BUTTON_OK;
                }
            }
        }
    } else {
        return BUTTON_STOP;
    }
}

void MC_set_motion(char motion) {
//    if (motion == last_motion) {
//        return;
//    } else {
        switch (motion) {
            case CMD_STOP:
                MC_BASE_COMMAND_OUT = STOP;
                return;
            case CMD_FORWARD:
                MC_BASE_COMMAND_OUT = FORWARD;
                return;
            case CMD_BACKWARD:
                MC_BASE_COMMAND_OUT = BACKWARD;
                return;
            case CMD_LEFT:
                MC_BASE_COMMAND_OUT = TURN_LEFT;
                return;
            case CMD_RIGHT:
                MC_BASE_COMMAND_OUT = TURN_RIGHT;
                return;
        }
//    }
}

void interrupt interrupt_handler() {
    if (RBIF) {
        last_RC_time = TMR1;
        last_RC_data = RB2;
        RBIF = 0;
    }
    
    if (CCP1IF) {
        if (ENA) {
            ENA = 0;
            ENB = 0;
            CCPR1 = CCPR1 + MC_LOW_PULSE;
        } else {
            ENA = 1;
            ENB = 1;
            CCPR1 = CCPR1 + MC_HIGH_PULSE;
        }
        CCP1IF = 0;
    }
}