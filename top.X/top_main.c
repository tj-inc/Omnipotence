/*
 * File:   top_main.c
 * Author: Zhou Zbou, Henry Teng
 *
 * Created on May 11, 2016, 10:11 AM
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
// TDP Module
#define TDP_LEFT RA0
#define TDP_CENTER RA1
#define TDP_RIGHT RA2
#define TDP_ONE_MIN 4135000   // 1min * 60s/min * 1000ms/s * 1000us/ms * 2/us / 30 = 4000000
enum TDP_States {LEFT90, RIGHT90, LEFT180, RIGHT180, TDP_Standby};

// WD Module
#define WD_LEFT RB0
#define WD_CENTER RB1
#define WD_RIGHT RB2
#define WD_Trigger_Width 10
#define WD_Collision_Threshold 174 // 30cm * 58us/cm

// MC Module
#define MC_OUT PORTD
#define NINTY_DEG_COUNT 8
#define ONEEIGHTY_DEG_COUNT 16
enum Motions {CMD_STOP, CMD_FORWARD, CMD_LEFT, CMD_RIGHT};
enum MC_States {Stop, Go_Forward, Turn_Left, Turn_Right};

// Mode
#define mode RC0

// Trigger
#define pull_trigger RC1
#define Trigger_Servo1 RC2
#define Trigger_Servo2 RC3
#define PWM_PERIOD 5000
#define PWM_INCREMENT 125
#define TWOFIFTY_MS 62500
#define TRIG_DELAY_MULTIPLIER 5
#define TRIG_COOLDOWN_MULTIPLIER 20
enum Pulse_Widths {MIN_HIGH = 1, MAX_HIGH = 5, TOTAL_WIDTH = 40};
enum Trigger_States {Trigger_StandBy, Trigger_Pulled, Trigger_CoolDown};

// RGB Module
#define R RC4
#define G RC5
#define B RC6
enum System_States {SYSTEM_INIT, SYSTEM_MANUAL, SYSTEM_SEARCHING, SYSTEM_ENGAGED};

// Function Prototypes
void interrupt interrupt_handler(void);

// Global Variables
// TDP Module
#define nTDP_Delay_Override RC7
char TDP_counter = 0;
char TDP_state = TDP_Standby;
bit last_direction = 0; // 0 is left, 1 is right

// WD Module
bit WD_last_left;
bit WD_last_center;
bit WD_last_right;
unsigned int last_WD_time;

// RGB Module
char system_state;

// Trigger
unsigned int high_pulse = MIN_HIGH;
char trigger_state = Trigger_StandBy;
char trigger_counter = 0;
char PWM_counter = 0;
bit trigger_under_auto = 0;

// MC Module

void main(void) {
    // Initialize RC0 and RC1 for mode and pull_trigger, and RC6:4 for RGB
    ANSEL = 0;
    ANSELH = 0;
    TRISC = 0b10000011;
    
    // MC Module
    TRISD = 0;
    
    // TDP Module
    TRISA = 0b111;
    
    // WD Module
    TRISB = 0b111;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
//    nRBPU = 0;
//    IOCB = 0b111;
//    WD_last_left = WD_LEFT;
//    WD_last_center = WD_CENTER;
//    WD_last_right = WD_RIGHT;
//    RBIF = 0;
//    RBIE = 1;
    
    // Init Timer 1
    TMR1GE = 0; TMR1ON = 1; 			//Enable TIMER1 (See Fig. 6-1 TIMER1 Block Diagram in PIC16F887 Data Sheet)
	TMR1CS = 0; 					//Select internal clock whose frequency is Fosc/4, where Fosc = 8 MHz
	T1CKPS1 = 1; T1CKPS0 = 1; 		 	//Set prescale to divide by 8 yielding a clock tick period of 4 microseconds
    
    // Init CCP1 for PWM
    CCP1M3 = 1; CCP1M2 = 0; CCP1M1 = 1; CCP1M0 = 0;
    CCP1IE = 1;
	CCP1IF = 0;
    
    // Init CCP2 for trigger timeout
    CCP2M3 = 1; CCP2M2 = 0; CCP2M1 = 1; CCP2M0 = 0;
    CCP2IE = 0; // We don't enable the interrupt yet
	CCP2IF = 0;
    
    // RGB Module
    system_state = SYSTEM_INIT;
    R = 1; G = 1; B = 0;
    // Delay 1 minute to prepare the sensors
    long i = 1;
    while (nTDP_Delay_Override) {
        if (i == TDP_ONE_MIN) {
            break;
        } else {
            i++;
        }
    }
    system_state = SYSTEM_MANUAL;
    
    // Turn on Interrupts
    CCPR1 = CCPR1 + 100;
    PEIE = 1;
	GIE = 1;
    while (1) {
        if (mode) {
            // This is auto mode
            if (TDP_CENTER) {
                system_state = SYSTEM_ENGAGED;
                MC_OUT = Go_Forward;
                TDP_state = TDP_Standby;
            } else {
                system_state = SYSTEM_SEARCHING;
                if (TDP_LEFT) {
                    MC_OUT = Turn_Right;
                    TDP_state = TDP_Standby;
                    last_direction = 0;
                    CCP2IE = 0;
                } else if (TDP_RIGHT) {
                    MC_OUT = Turn_Left;
                    TDP_state = TDP_Standby;
                    last_direction = 1;
                    CCP2IE = 0;
                } else {
                    // TDP FSM
                    switch (TDP_state) {
                        case TDP_Standby:
                            if (last_direction) {
                                TDP_state = RIGHT90;
                            } else {
                                TDP_state = LEFT90;
                            }
                            TDP_counter = 0;
                            break;
                        case LEFT90:
                            MC_OUT = Turn_Left;
                            if (CCP2IE == 0) {
                                CCPR2 = CCPR2 + TWOFIFTY_MS;
                                CCP2IE = 1;
                            }
                            break;
                        case RIGHT90:
                            MC_OUT = Turn_Right;
                            if (CCP2IE == 0) {
                                CCPR2 = CCPR2 + TWOFIFTY_MS;
                                CCP2IE = 1;
                            }
                            break;
                        case LEFT180:
                            MC_OUT = Turn_Left;
                            if (CCP2IE == 0) {
                                CCPR2 = CCPR2 + TWOFIFTY_MS;
                                CCP2IE = 1;
                            }
                            break;
                        case RIGHT180:
                            MC_OUT = Turn_Right;
                            if (CCP2IE == 0) {
                                CCPR2 = CCPR2 + TWOFIFTY_MS;
                                CCP2IE = 1;
                            }
                            break;
                    }
                }
            }
        } else {
            system_state = SYSTEM_MANUAL;
            TDP_state = TDP_Standby;
        }
        
        // System Main FSM
        switch (system_state) {
            case SYSTEM_MANUAL:
                R = 0; G = 1; B = 0;
                break;
            case SYSTEM_SEARCHING:
                trigger_under_auto = 0;
                R = 0; G = 1; B = 1;
                break;
            case SYSTEM_ENGAGED:
                trigger_under_auto = 1;
                R = 1; G = 0; B = 0;
                break;
            default:
                R = 1; G = 1; B = 0;
                break;
        }
        
        // Trigger FSM
        switch (trigger_state) {
            case Trigger_StandBy:
                if ((mode & trigger_under_auto) | (~mode & pull_trigger)) {
                    trigger_state = Trigger_Pulled;
                    trigger_counter = 0;
                }
                high_pulse = MIN_HIGH;
                break;
            case Trigger_Pulled:
                if (CCP2IE == 0) {
                    CCPR2 = CCPR2 + TWOFIFTY_MS;
                    CCP2IE = 1;
                }
                high_pulse = MAX_HIGH;
                break;
            case Trigger_CoolDown:
                if (CCP2IE == 0) {
                    CCPR2 = CCPR2 + TWOFIFTY_MS;
                    CCP2IE = 1;
                }
                high_pulse = MIN_HIGH;
                break;
        }
    }
}

void interrupt interrupt_handler() {
    if (CCP1IF) {
        CCPR1 = CCPR1 + PWM_INCREMENT;
        if (PWM_counter < high_pulse) {
            Trigger_Servo1 = 1;
        } else {
            Trigger_Servo1 = 0;
        }
        
        if (high_pulse == MAX_HIGH) {
            if (PWM_counter < MIN_HIGH) {
                Trigger_Servo2 = 1;
            } else {
                Trigger_Servo2 = 0;
            }
        } else {
            if (PWM_counter < MAX_HIGH) {
                Trigger_Servo2 = 1;
            } else {
                Trigger_Servo2 = 0;
            }
        }
        
        if (PWM_counter == TOTAL_WIDTH) {
            PWM_counter = 0;
        } else {
            PWM_counter++;
        }
        CCP1IF = 0;
    }
    
    if (CCP2IF) {
        CCPR2 = CCPR2 + TWOFIFTY_MS;
        switch (trigger_state) {  // Do I know the current state is always right when I'm here
            case Trigger_Pulled:
                if (trigger_counter == TRIG_DELAY_MULTIPLIER) {
                    trigger_state = Trigger_CoolDown;
                    trigger_counter = 0;
                    CCP2IE = 0;
                } else {
                    trigger_counter++;
                }
                break;
            case Trigger_CoolDown:
                if (trigger_counter == TRIG_COOLDOWN_MULTIPLIER) {
                    trigger_state = Trigger_StandBy;
                    CCP2IE = 0;
                } else {
                    trigger_counter++;
                }
                break;
        }
        switch (TDP_state) {
            case LEFT90:
                if (TDP_counter == NINTY_DEG_COUNT) {
                    TDP_state = RIGHT180;
                    TDP_counter = 0;
                    CCP2IE = 0;
                } else {
                    TDP_counter++;
                }
                break;
            case RIGHT90:
                if (TDP_counter == NINTY_DEG_COUNT) {
                    TDP_state = LEFT180;
                    TDP_counter = 0;
                    CCP2IE = 0;
                } else {
                    TDP_counter++;
                }
                break;
            case LEFT180:
                if (TDP_counter == ONEEIGHTY_DEG_COUNT) {
                    TDP_state = RIGHT180;
                    TDP_counter = 0;
                    CCP2IE = 0;
                } else {
                    TDP_counter++;
                }
                break;
            case RIGHT180:
                if (TDP_counter == ONEEIGHTY_DEG_COUNT) {
                    TDP_state = LEFT180;
                    TDP_counter = 0;
                    CCP2IE = 0;
                } else {
                    TDP_counter++;
                }
                break;
        }
        CCP2IF = 0;
    }
    
    if (RBIF) {
        last_WD_time = TMR1;
        WD_last_left = WD_LEFT;
        WD_last_center = WD_CENTER;
        WD_last_right = WD_RIGHT;
        RBIF = 0;
    }
}
