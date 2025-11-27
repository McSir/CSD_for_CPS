#include "lab04.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "math.h"

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "dac.h"

// signal parameter

#define freq 1 // Hz
#define samRate 300 // Hz
#define Vmin 1 // Volts
#define Vmax 3 // Volts

#define PS 256 // timer 3 prescaler
#define FREQ_TO_TICKS (FCY / (freq * PS))

/*
 * Timer code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

uint32_t t = 0; // global time variable
uint32_t Vout = 0; 

void timer_initialize()
{
    CLEARBIT(T3CONbits.TON); // Disable Timer
    CLEARBIT(T3CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T3CONbits.TGATE); // Disable Gated Timer mode
    TMR3 = 0x00; // Clear timer register
    T3CONbits.TCKPS = PS; // Select Prescaler
    PR3 = FREQ_TO_TICKS; // Load the period value
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    CLEARBIT(IFS0bits.T3IF); // Clear Timer3 Interrupt Flag
    SETBIT(IEC0bits.T3IE); // Enable Timer3 interrupt
    SETBIT(T3CONbits.TON); // Start Timer
}

// interrupt service routine
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
    //_T3IF = 0;
    
    LED1_PORT ^= 0x01;
    Nop();
    
    Vout = (((Vmax-Vmin)/2) * sin(2*3.14*freq*t)) + (Vmax-Vmin);
    
    dac_convert_milli_volt(Vout); // update DAC
    
    IFS0bits.T3IF = 0; // Clear the interrupt flag
}

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: 19");
    
    while(TRUE) { }
}
