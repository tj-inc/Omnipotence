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
enum TDP_States {LEFT90, RIGHT90, LEFT180, RIGHT180, TDP_Standby, TDP_Engaged, TDP_Evade_Left1, TDP_Evade_Left2, TDP_Evade_Center1, TDP_Evade_Center2, TDP_Evade_Right1, TDP_Evade_Right2};

// WD Module
#define WD_LEFT RB0
#define WD_CENTER RB1
#define WD_RIGHT RB2
#define WD_Trigger_Width 10
#define WD_Collision_Threshold 174 // 30cm * 58us/cm
#define WD_10us 246
enum WD_States {Left_Out, Left_In, Center_Out, Center_In, Right_Out, Right_In};

// MC Module
#define MC_OUT PORTD
#define FORTYFIVE_DEG_COUNT 4
#define NINTY_DEG_COUNT 8
#define ONEEIGHTY_DEG_COUNT 16
#define ENGAGED_DELAY 2
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
#define TRIG_COOLDOWN_MULTIPLIER 10
enum Pulse_Widths {MIN_HIGH = 1, MAX_HIGH = 5, TOTAL_WIDTH = 40};
enum Trigger_States {Trigger_StandBy, Trigger_Pulled, Trigger_CoolDown};

// RGB Module
#define R RC4
#define G RC5
#define B RC6
enum System_States {SYSTEM_INIT, SYSTEM_MANUAL, SYSTEM_SEARCHING, SYSTEM_ENGAGED};

// Function Prototypes
void TDP_evade_left(void);
void TDP_evade_center(void);
void TDP_evade_right(void);
void interrupt interrupt_handler(void);

// Global Variables
// TDP Module
#define nTDP_Delay_Override RC7
char TDP_counter = 0;
char TDP_state = TDP_Standby;
char TDP_saved_state;
char TDP_evade_counter = 0;
bit last_direction = 0; // 0 is left, 1 is right

// WD Module
bit WD_last_left;
bit WD_last_center;
bit WD_last_right;
unsigned int last_RE_time;
unsigned int last_FE_time;
char WD_state;
bit WD_feedback_received = 0;
bit WD_probe_sent = 0;
bit WD_probe_finished = 0;

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
    TRISB = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    
    // Init Timer 0
    T0CS = 0;
    PSA = 0;
    OPTION_REG &= 0b11111000; // Prescaler of 2
    
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
            switch (WD_state) {
                case Left_Out:
                    TRISB0 = 0;
                    if (WD_probe_sent) {
                        if (WD_probe_finished) {
                            WD_LEFT = 0;
                            T0IE = 0;
                            WD_state = Left_In;
                            IOCB0 = 1;
                            WD_LEFT = WD_LEFT;
                            RBIF = 0;
                            RBIE = 1;
                        }
                    } else {
                        TMR0 = WD_10us;
                        WD_LEFT = 1;
                        T0IF = 0;
                        T0IE = 1;
                        WD_probe_sent = 1;
                    }
                    break;
                case Left_In:
                    TRISB0 = 1;
                    if (WD_feedback_received) {
                        IOCB = 0;
                        RBIE = 0;
                        if (last_FE_time - last_RE_time > WD_Collision_Threshold) {
                            TDP_evade_left();
                        }
                        WD_state = Center_Out;
                    }
                    break;
                case Center_Out:
                    TRISB1 = 0;
                    if (WD_probe_sent) {
                        if (WD_probe_finished) {
                            WD_CENTER = 0;
                            T0IE = 0;
                            WD_state = Center_In;
                            IOCB1 = 1;
                            WD_CENTER = WD_CENTER;
                            RBIF = 0;
                            RBIE = 1;
                        }
                    } else {
                        TMR0 = WD_10us;
                        WD_CENTER = 1;
                        T0IF = 0;
                        T0IE = 1;
                        WD_probe_sent = 1;
                    }
                    break;
                case Center_In:
                    TRISB1 = 1;
                    if (WD_feedback_received) {
                        IOCB = 0;
                        RBIE = 0;
                        if (last_FE_time - last_RE_time > WD_Collision_Threshold) {
                            TDP_evade_center();
                        }
                        WD_state = Right_Out;
                    }
                    break;
                case Right_Out:
                    TRISB2 = 0;
                    if (WD_probe_sent) {
                        if (WD_probe_finished) {
                            WD_RIGHT = 0;
                            T0IE = 0;
                            WD_state = Right_In;
                            IOCB2 = 1;
                            WD_RIGHT = WD_RIGHT;
                            RBIF = 0;
                            RBIE = 1;
                        }
                    } else {
                        TMR0 = WD_10us;
                        WD_RIGHT = 1;
                        T0IF = 0;
                        T0IE = 1;
                        WD_probe_sent = 1;
                    }
                    break;
                case Right_In:
                    TRISB2 = 1;
                    if (WD_feedback_received) {
                        IOCB = 0;
                        RBIE = 0;
                        if (last_FE_time - last_RE_time > WD_Collision_Threshold) {
                            TDP_evade_right();
                        }
                        WD_state = Left_Out;
                    }
                    break;
            }
            
            if (TDP_CENTER) {
                system_state = SYSTEM_ENGAGED;
                MC_OUT = Go_Forward;
                TDP_state = TDP_Standby;
                TDP_counter = 0;
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
                        case TDP_Engaged:
                            MC_OUT = Go_Forward;
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

void TDP_evade_left() {
    CCPR2 = CCPR2 + TWOFIFTY_MS;
    TDP_saved_state = TDP_state;
    TDP_state = TDP_Evade_Left1;
    TDP_evade_counter = 0;
    if (CCP2IE == 0) {
        CCP2IE = 1;
    }
    MC_OUT = Turn_Right;
}

void TDP_evade_center() {
    CCPR2 = CCPR2 + TWOFIFTY_MS;
    TDP_saved_state = TDP_state;
    TDP_state = TDP_Evade_Center1;
    TDP_evade_counter = 0;
    if (CCP2IE == 0) {
        CCP2IE = 1;
    }
    MC_OUT = Turn_Left;
}

void TDP_evade_right() {
    CCPR2 = CCPR2 + TWOFIFTY_MS;
    TDP_saved_state = TDP_state;
    TDP_state = TDP_Evade_Right1;
    TDP_evade_counter = 0;
    if (CCP2IE == 0) {
        CCP2IE = 1;
    }
    MC_OUT = Turn_Left;
}

void interrupt interrupt_handler() {
    if (T0IF) {
        WD_probe_finished = 1;
        T0IF = 0;
    }
    
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
        switch (trigger_state) {
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
            case TDP_Engaged:
                 if (TDP_counter == ENGAGED_DELAY) {
                    TDP_state = TDP_Standby;
                    TDP_counter = 0;
                    CCP2IE = 0;
                } else {
                    TDP_counter++;
                }
                break;
            case TDP_Evade_Left1:
                if (TDP_evade_counter == FORTYFIVE_DEG_COUNT) {
                    TDP_state = TDP_Evade_Left2;
                    TDP_evade_counter = 0;
                    MC_OUT = Go_Forward;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
            case TDP_Evade_Left2:
                if (TDP_evade_counter == FORTYFIVE_DEG_COUNT) {
                    TDP_state = TDP_saved_state;
                    TDP_evade_counter = 0;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
            case TDP_Evade_Center1:
                if (TDP_evade_counter == NINTY_DEG_COUNT) {
                    TDP_state = TDP_Evade_Center2;
                    TDP_evade_counter = 0;
                    MC_OUT = Go_Forward;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
            case TDP_Evade_Center2:
                if (TDP_evade_counter == FORTYFIVE_DEG_COUNT) {
                    TDP_state = TDP_saved_state;
                    TDP_evade_counter = 0;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
            case TDP_Evade_Right1:
                if (TDP_evade_counter == FORTYFIVE_DEG_COUNT) {
                    TDP_state = TDP_Evade_Right2;
                    TDP_evade_counter = 0;
                    MC_OUT = Go_Forward;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
            case TDP_Evade_Right2:
                if (TDP_evade_counter == FORTYFIVE_DEG_COUNT) {
                    TDP_state = TDP_saved_state;
                    TDP_evade_counter = 0;
                } else {
                    TDP_evade_counter++;
                }
                CCPR2 = CCPR2 + TWOFIFTY_MS;
                break;
        }
        CCP2IF = 0;
    }
    
    if (RBIF) {
        last_RE_time = last_FE_time;
        last_FE_time = TMR1;
        WD_last_left = WD_LEFT;
        WD_last_center = WD_CENTER;
        WD_last_right = WD_RIGHT;
        RBIF = 0;
    }
}