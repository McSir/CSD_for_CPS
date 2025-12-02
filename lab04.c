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

static volatile uint32_t g_ms = 0;
uint32_t Vout = 0; 

// signal parameter
#define freq 1 // Hz
#define samRate 300 // Hz
#define Vmin 1 // Volts
#define Vmax 3 // Volts

#define PI 3.14

#define PS 256 // timer 3 prescaler
#define FREQ_TO_TICKS (FCY / (freq * PS))

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x02
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define T3_PRESCALE_RATIO 64UL
#define T3_PR_1MS ((uint16_t)((FCY / T3_PRESCALE_RATIO / 1000UL) - 1UL))

void timer_initialize()
{
    
    CLEARBIT(T3CONbits.TON); // Disable Timer
    CLEARBIT(T3CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T3CONbits.TGATE); // Disable Gated Timer mode
    T3CONbits.TCKPS = 0b11; // Select Prescaler
    PR3 = 65535;//FREQ_TO_TICKS; // Load the period value
    TMR3 = 0x00; // Clear timer register
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    CLEARBIT(IFS0bits.T3IF); // Clear Timer3 Interrupt Flag
    SETBIT(IEC0bits.T3IE); // Enable Timer3 interrupt
    SETBIT(T3CONbits.TON); // Start Timer

}

// interrupt service routine
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
    _T3IF = 0;
    g_ms = g_ms + 3;
}

/*
 * main loop
 */

uint32_t start;

void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: %lu", (unsigned long)g_ms);
    
    while(TRUE)
    {
        LED1_PORT ^= 0x01;
        Nop();

        Vout = ((((Vmax-Vmin)/2) * sin(2*PI*freq*g_ms)) + ((Vmax+Vmin)/2));

        dac_convert_milli_volt(Vout); // update DAC
        
        //500 ms
        start = g_ms;
        while ((unsigned long)(g_ms - start) < 100UL)
            ;
    }
}