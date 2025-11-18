#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/* simple millisecond counter driven by Timer1 */
static volatile unsigned long g_ms = 0;

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
    DAC_SDI_AD1CFG  = 1;    // RB10 as digital
    DAC_SCK_AD1CFG  = 1;    // RB11 as digital
    DAC_LDAC_AD1CFG = 1;    // RB13 as digital

    DAC_SDI_AD2CFG  = 1;    // RB10 as digital
    DAC_SCK_AD2CFG  = 1;    // RB11 as digital
    DAC_LDAC_AD2CFG = 1;    // RB13 as digital
    
    // set RD8, RB10, RB11, RB13 as output pins
    DAC_CS_TRIS   = 0;      // RD8  output (CS)
    DAC_SDI_TRIS  = 0;      // RB10 output (SDI)
    DAC_SCK_TRIS  = 0;      // RB11 output (SCK)
    DAC_LDAC_TRIS = 0;      // RB13 output (LDAC)
    
    // set default state: CS=1, SCK=0, SDI=0, LDAC=1
    DAC_CS_PORT   = 1;      // deselect DAC
    DAC_SCK_PORT  = 0;      // clock idle low
    DAC_SDI_PORT  = 0;      // data low
    DAC_LDAC_PORT = 1;      // LDAC inactive (no latch)
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

    // configure timer (Timer1 with external 32.768 kHz crystal, ~1 ms period)
    T1CONbits.TON   = 0;          // stop timer during configuration
    T1CONbits.TCS   = 1;          // external clock source (RTC)
    T1CONbits.TCKPS = TCKPS_1;    // prescaler 1:1

    TMR1 = 0;

    // Period register for ~1 ms:
    // T = (PR1 + 1) / 32768  -> PR1 ≈ 31 => 32 / 32768 ≈ 0.000976 s
    PR1 = (uint32_t)(FCY_EXT / 1000UL - 1UL);

    // configure Timer1 interrupt
    _T1IF = 0;                    // clear interrupt flag
    _T1IP = 3;                    // priority
    _T1IE = 1;                    // enable interrupt

    // start timer
    T1CONbits.TON = 1;
}

// interrupt service routine?
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;        // clear interrupt flag
    g_ms++;           // increment millisecond counter
}

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: test19");
    
    // approximate DAC codes for 1.0 V, 2.5 V, 3.5 V (assuming ~4 V full scale)
    unsigned int value_1V   = 1000;          // or whatever you're using that gives 1.0 V

    // derive the others *from* value_1V so they keep the right ratio:
    unsigned int value_2_5V = value_1V * 5 / 2;  // 2.5 * value_1V
    unsigned int value_3_5V = value_1V * 7 / 2;  // 3.5 * value_1V
    
    while(TRUE)
    {
        uint16_t cmd;
        uint16_t i;
        uint32_t start;

        /************ 1. Output ~1.0 V ************/
        
        LED1_PORT ^= 0x01;
        Nop();
        
        cmd = 0;
        // MCP4822 command bits:
        // bit15: A/B = 0 (channel A)
        // bit14: BUF = 1 (buffered)
        // bit13: GA  = 0 (2x gain -> full scale ~4.096V)
        // bit12: SHDN= 1 (active)
        cmd |= (1u << 14) | (0u << 13) | (1u << 12);
        cmd |= (value_1V & 0x0FFFu);


        // start frame: CS low
        DAC_CS_PORT = 0;
        Nop();

        // shift out 16 bits, MSB first
        for (i = 0; i < 16; i++)
        {
            if (cmd & 0x8000u)
                DAC_SDI_PORT = 1;
            else
                DAC_SDI_PORT = 0;

            Nop();
            DAC_SCK_PORT = 1;    // rising edge clocks bit
            Nop();
            DAC_SCK_PORT = 0;    // fall back low
            Nop();

            cmd <<= 1;           // next bit
        }

        // end frame
        DAC_CS_PORT = 1;
        Nop();

        // latch new value (LDAC low pulse)
        DAC_LDAC_PORT = 0;
        Nop();
        DAC_LDAC_PORT = 1;

        // wait 500 ms using Timer1
        start = g_ms;
        while ((unsigned long)(g_ms - start) < 500UL)
            ;

        /************ 2. Output ~2.5 V ************/
        
        LED1_PORT ^= 0x01;
        Nop();
        
        cmd = 0;
        cmd |= (1u << 14) | (0u << 13) | (1u << 12);
        cmd |= (value_2_5V & 0x0FFFu);

        DAC_CS_PORT = 0;
        Nop();

        for (i = 0; i < 16; i++)
        {
            if (cmd & 0x8000u)
                DAC_SDI_PORT = 1;
            else
                DAC_SDI_PORT = 0;

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

        // wait 2000 ms
        start = g_ms;
        while ((unsigned long)(g_ms - start) < 2000UL)
            ;

        /************ 3. Output ~3.5 V ************/
        
        LED1_PORT ^= 0x01;
        Nop();
        
        cmd = 0;
        cmd |= (1u << 14) | (0u << 13) | (1u << 12);
        cmd |= (value_3_5V & 0x0FFFu);

        DAC_CS_PORT = 0;
        Nop();

        for (i = 0; i < 16; i++)
        {
            if (cmd & 0x8000u)
                DAC_SDI_PORT = 1;
            else
                DAC_SDI_PORT = 0;

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

        // wait 1000 ms
        start = g_ms;
        while ((unsigned long)(g_ms - start) < 1000UL)
            ;

        // (optional) here you could toggle an LED once per full cycle
        // e.g. led_toggle(LED1);  if your led.h provides that function
    }
}
