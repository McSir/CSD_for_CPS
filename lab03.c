#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "math.h"

static volatile uint32_t g_ms = 0;

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    DAC_SDI_AD1CFG  = 1;
    DAC_SCK_AD1CFG  = 1;
    DAC_LDAC_AD1CFG = 1;

    DAC_SDI_AD2CFG  = 1;
    DAC_SCK_AD2CFG  = 1;
    DAC_LDAC_AD2CFG = 1;
    
    // set RD8, RB10, RB11, RB13 as output pins
    DAC_CS_TRIS   = 0;
    DAC_SDI_TRIS  = 0;
    DAC_SCK_TRIS  = 0;
    DAC_LDAC_TRIS = 0;
    
    // set default state: CS=1, SCK=0, SDI=0, LDAC=1
    DAC_CS_PORT   = 1;
    DAC_SCK_PORT  = 0;
    DAC_SDI_PORT  = 0;
    DAC_LDAC_PORT = 1;
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);

    T1CONbits.TON   = 0;          // stop timer during configuration
    T1CONbits.TCS   = 1;          // external clock source (RTC)
    T1CONbits.TCKPS = TCKPS_1;    // prescaler 1:1

    TMR1 = 0;

    // Period register for 1 ms:
    PR1 = (uint32_t)(FCY_EXT / 1000UL - 1UL);

    _T1IF = 0;                    // clear interrupt flag
    _T1IP = 3;                    // priority
    _T1IE = 1;                    // enable interrupt

    T1CONbits.TON = 1;
}

// interrupt service routine
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;
    g_ms++;
}

/*
 * main loop
 */

#define freq 1 // Hz
#define samRate 300 // Hz
#define Vmin 1 // Volts
#define Vmax 3 // Volts

#define PI 3.14

uint32_t Vout = 0; 

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: 19");
    
    while(TRUE)
    {
        uint16_t cmd;
        uint16_t i;
        uint32_t start;

        //1.0 V
        
        LED1_PORT ^= 0x01;
        Nop();
        
        cmd = 0;
        cmd |= (0u << 13) | (1u << 12);  // GA -> 1x 
                                         // SHDN -> 1 to activate DAC
        
        Vout = ((((Vmax-Vmin)/2) * sin(2*PI*freq*g_ms)) + ((Vmax+Vmin)/2))*1000;
        cmd |= (Vout & 0x0FFFu); // add data bits

        DAC_CS_PORT = 0;
        Nop();

        for (i = 0; i < 16; i++)
        {
            if (cmd & 0x8000u) // depending on bit A/B(15) in cmd
                DAC_SDI_PORT = 1; // write to DAC B
            else
                DAC_SDI_PORT = 0; // write to DAC A

            Nop();
            DAC_SCK_PORT = 1;  
            Nop();
            DAC_SCK_PORT = 0;
            Nop();

            cmd <<= 1;
        }

        DAC_CS_PORT = 1;
        Nop();

        DAC_LDAC_PORT = 0;
        Nop();
        DAC_LDAC_PORT = 1;

        //500 ms
        start = g_ms;
        while ((unsigned long)(g_ms - start) < 100UL)
            ;
    }
}
