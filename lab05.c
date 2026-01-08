#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * PWM code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 3800 //1000
#define PWM_MID_US 3700 //1500
#define PWM_MAX_US 3600 //2000
#define PWM_CYC_US 4000 //20000

#define XMAX 730
#define XMIN 60

#define YMAX 645
#define YMIN 95


uint16_t Xmapping(uint16_t p){
    return p*(PWM_MIN_US-PWM_MAX_US)/(XMAX - XMIN) + PWM_MAX_US - XMIN*(PWM_MIN_US-PWM_MAX_US)/(XMAX - XMIN);
}

uint16_t Ymapping(uint16_t p){
    return p*(PWM_MIN_US-PWM_MAX_US)/(YMAX - YMIN) + PWM_MAX_US - YMIN*(PWM_MIN_US-PWM_MAX_US)/(YMAX - YMIN);
}

void servo_init(uint8_t x, uint8_t y)
{
    if(x)
    {
        // Setup Timer 2
        CLEARBIT(T2CONbits.TON); // Disable Timer
        CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
        TMR2 = 0x00; // Clear timer register
        T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
        CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
        PR2 = PWM_CYC_US; // Set timer period 20 ms:
                    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
        // Setup OC8
        CLEARBIT(TRISDbits.TRISD7); // Set OC8 as output
        OC8R = PWM_MIN_US; // Set the initial duty cycle to 1 ms
        OC8RS = PWM_MIN_US; // Load OCRS: next pwm duty cycle
        OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
        SETBIT(T2CONbits.TON); // Turn Timer 2 on
    }
    
    if(y)
    {
        // Setup Timer 1
        CLEARBIT(T1CONbits.TON); // Disable Timer
        CLEARBIT(T1CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T1CONbits.TGATE); // Disable Gated Timer mode
        TMR1 = 0x00; // Clear timer register
        T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T1IF); // Clear Timer1 interrupt status flag
        CLEARBIT(IEC0bits.T1IE); // Disable Timer1 interrupt enable control bit
        PR1 = PWM_CYC_US; // Set timer period 20 ms:
                    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
        // Setup OC7
        CLEARBIT(TRISDbits.TRISD6); // Set OC7 as output
        OC7R = PWM_MIN_US; // Set the initial duty cycle to 1 ms
        OC7RS = PWM_MIN_US; // Load OCRS: next pwm duty cycle
        OC7CON = 0x0006; // Set OC7: PWM, no fault check, Timer2
        SETBIT(T1CONbits.TON); // Turn Timer 1 on
    }
}

void servo_set(uint8_t x, uint8_t y, uint8_t dc)
{
    if(x)
    {
        // Setup Timer 2
        CLEARBIT(T2CONbits.TON); // Disable Timer
        CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
        TMR2 = 0x00; // Clear timer register
        T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
        CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
        PR2 = PWM_CYC_US; // Set timer period 20 ms:
                    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
        // Setup OC8
        CLEARBIT(TRISDbits.TRISD7); // Set OC8 as output
        OC8R = dc; // Set the initial duty cycle to 1 ms
        OC8RS = dc; // Load OCRS: next pwm duty cycle
        OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
        SETBIT(T2CONbits.TON); // Turn Timer 2 on
    }
    
    if(y)
    {
        // Setup Timer 1
        CLEARBIT(T1CONbits.TON); // Disable Timer
        CLEARBIT(T1CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T1CONbits.TGATE); // Disable Gated Timer mode
        TMR1 = 0x00; // Clear timer register
        T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T1IF); // Clear Timer1 interrupt status flag
        CLEARBIT(IEC0bits.T1IE); // Disable Timer1 interrupt enable control bit
        PR1 = PWM_CYC_US; // Set timer period 20 ms:
                    // 4000 = 20*10^-3 * 12.8*10^6 * 1/64
        // Setup OC7
        CLEARBIT(TRISDbits.TRISD6); // Set OC7 as output
        OC7R = dc; // Set the initial duty cycle to 1 ms
        OC7RS = dc; // Load OCRS: next pwm duty cycle
        OC7CON = 0x0006; // Set OC7: PWM, no fault check, Timer2
        SETBIT(T1CONbits.TON); // Turn Timer 1 on
    }
}

/*
 * touch screen code
 */

void touchscreen_init()
{
    // Set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output
    // Set up the I/O pins E1, E2, E3 so that the touchscreen X-coordinate pin connects to the ADC
    CLEARBIT(PORTEbits.RE1);
    SETBIT(PORTEbits.RE2);
    SETBIT(PORTEbits.RE3);
    
    
}


/*
 * main loop
 */


void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: 19");
    
    // initialize touchscreen
    
    // initialize servos
    servo_init(1, 1);
    servo_set(1, 1, PWM_MAX_US);
    
    while(TRUE) {
    
    }
}
