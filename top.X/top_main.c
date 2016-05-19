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
#define TDP_ONE_MIN 4437800   // 1min * 60s/min * 1000ms/s * 1000us/ms * 2/us / 30 = 4000000

// WD Module
#define WD_LEFT RB0
#define WD_CENTER RB1
#define WD_RIGHT RB2
#define WD_Trigger_Width 10
#define WD_Collision_Threshold 174 // 30cm * 58us/cm

// MC Module

// Mode
#define mode RC0

// Trigger
#define pull_trigger RC1
#define PWM_PERIOD 5000
#define TWOFIFTY_MS 62500
#define TRIG_DELAY_MULTIPLIER 5
enum Pulse_Widths {MIN_HIGH = 125, FORTYFIVE_HIGH = 251, NINTY_HIGH = 376, ONETHIRTYFIVE_HIGH = 502, MAX_HIGH = 627};
enum Trigger_States {Trigger_StandBy, Trigger_Pulled, Trigger_CoolDown};

// RGB Module
#define RGB_R RC4
#define RGB_G RC5
#define RGB_B RC6
enum System_States {SYSTEM_INIT, SYSTEM_MANUAL, SYSTEM_SEARCHING, SYSTEM_ENGAGED};

// Function Prototypes
void interrupt interrupt_handler(void);

// Global Variables
// TDP Module

// WD Module
bit WD_last_left;
bit WD_last_center;
bit WD_last_right;
unsigned int last_WD_time;

// RGB Module
char system_state = SYSTEM_INIT;

// Trigger
unsigned int high_pulse = MIN_HIGH;
char trigger_state = Trigger_StandBy;
char counter = 0;

//// Async Task Management Module
//struct Task delays[11];
//char task_index = 1;

void main(void) {
    // Initialize RC0 and RC1 for mode and pull_trigger, and RC6:4 for RGB
    ANSEL = 0;
    ANSELH = 0;
//    TRISC = 0b1110011;
    TRISC = 0xFF;
    TRISC2 = 0;
    
    // TDP Module
    TRISA = 0b111;
    
    // WD Module
    TRISB = 0b111;
//    nRBPU = 0;
//    IOCB = 0b111;
    WD_last_left = WD_LEFT;
    WD_last_center = WD_CENTER;
    WD_last_right = WD_RIGHT;
    RBIF = 0;
    RBIE = 1;
    
    // Init Timer 1
    TMR1GE = 0; TMR1ON = 1; 			//Enable TIMER1 (See Fig. 6-1 TIMER1 Block Diagram in PIC16F887 Data Sheet)
	TMR1CS = 0; 					//Select internal clock whose frequency is Fosc/4, where Fosc = 8 MHz
	T1CKPS1 = 1; T1CKPS0 = 1; 		 	//Set prescale to divide by 8 yielding a clock tick period of 4 microseconds
    
    // Init CCP1 for PWM
    CCP1M3 = 0; CCP1M2 = 0; CCP1M1 = 1; CCP1M0 = 0;
    CCP1IE = 1;
	CCP1IF = 0;
    
    // Init CCP2 for trigger timeout
    CCP2M3 = 1; CCP2M2 = 0; CCP2M1 = 1; CCP2M0 = 0;
    CCP2IE = 0; // We don't enable the interrupt yet
	CCP2IF = 0;
    
    // Delay 1 minute to prepare the sensors
//    for (long i = 1; i < TDP_ONE_MIN; i++);
    
    // Turn on Interrupts
    CCPR1 = CCPR1 + 100;
    PEIE = 1;
	GIE = 1;
    while (1) {
        if (mode) {
            // This is auto mode
            // Trigger FSM
            
        }
        
        switch (trigger_state) {
            case Trigger_StandBy:
                if (pull_trigger == 0) {
                    trigger_state = Trigger_Pulled;
                    counter = 0;
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
        if (RC2) {
            CCPR1 = CCPR1 + high_pulse;
        } else {
            CCPR1 = CCPR1 + PWM_PERIOD - high_pulse;
        }
        CCP1IF = 0;
    }
    
    if (CCP2IF) {
        switch (trigger_state) {  // Do I know the current state is always right when I'm here
            case Trigger_Pulled:
                if (counter == TRIG_DELAY_MULTIPLIER) {
                    trigger_state = Trigger_CoolDown;
                    counter = 0;
                    CCP2IE = 0;
                } else {
                    counter++;
                }
                break;
            case Trigger_CoolDown:
                if (counter == TRIG_DELAY_MULTIPLIER) {
                    trigger_state = Trigger_StandBy;
                    CCP2IE = 0;
                } else {
                    counter++;
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
