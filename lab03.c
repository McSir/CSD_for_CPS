#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include <stdint.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS   TRISDbits.TRISD8
#define DAC_SDI_TRIS  TRISBbits.TRISB10
#define DAC_SCK_TRIS  TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT   PORTDbits.RD8
#define DAC_SDI_PORT  PORTBbits.RB10
#define DAC_SCK_PORT  PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG  AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG  AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG  AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG  AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

/* build a 16-bit command word for the MCP4822 (channel A) */
static uint16_t dac_build_cmd(uint16_t value)
{
    uint16_t cmd = 0;

    /* bit 15: A/B = 0 (channel A)   */
    /* bit 14: BUF = 1 (buffered)    */
    /* bit 13: GA  = 1 (1x gain)     */
    /* bit 12: SHDN= 1 (active)      */
    cmd |= (0u << 15);
    cmd |= (1u << 14);
    cmd |= (1u << 13);
    cmd |= (1u << 12);

    /* lower 12 bits: DAC data */
    cmd |= (value & 0x0FFFu);

    return cmd;
}

/* bit-bang one 16-bit word to the DAC (MSB first) */
static void dac_send_word(uint16_t word)
{
    int8_t i;

    /* start frame: CS low */
    DAC_CS_PORT = 0;
    Nop();

    for (i = 15; i >= 0; i--)
    {
        /* put current bit on SDI */
        if (word & (1u << i))
        {
            DAC_SDI_PORT = 1;
        }
        else
        {
            DAC_SDI_PORT = 0;
        }

        /* clock rising edge */
        DAC_SCK_PORT = 1;
        Nop();
        /* clock falling edge */
        DAC_SCK_PORT = 0;
        Nop();
    }

    /* end frame: CS high */
    DAC_CS_PORT = 1;
    Nop();

    /* latch new value at LDAC (active low pulse) */
    DAC_LDAC_PORT = 0;
    Nop();
    DAC_LDAC_PORT = 1;
}

/* convenience function: write a 12-bit value to channel A */
static void dac_set_raw(uint16_t value)
{
    uint16_t cmd = dac_build_cmd(value);
    dac_send_word(cmd);
}

void dac_initialize()
{
    /* set AN10, AN11, AN13 to digital mode
       this means AN10->RB10, AN11->RB11, AN13->RB13
       see datasheet section on AD1PCFGL/AD2PCFGL
    */

    DAC_SDI_AD1CFG  = 1;   /* RB10 digital */
    DAC_SCK_AD1CFG  = 1;   /* RB11 digital */
    DAC_LDAC_AD1CFG = 1;   /* RB13 digital */

    DAC_SDI_AD2CFG  = 1;   /* RB10 digital */
    DAC_SCK_AD2CFG  = 1;   /* RB11 digital */
    DAC_LDAC_AD2CFG = 1;   /* RB13 digital */

    /* set RD8, RB10, RB11, RB13 as output pins */
    DAC_CS_TRIS   = 0;     /* RD8 output  */
    DAC_SDI_TRIS  = 0;     /* RB10 output */
    DAC_SCK_TRIS  = 0;     /* RB11 output */
    DAC_LDAC_TRIS = 0;     /* RB13 output */

    /* default pin states:
       CS   = 1 (inactive)
       SCK  = 0 (idle low)
       SDI  = 0
       LDAC = 1 (no update)
    */
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

/* we choose a 10 ms software tick */
#define TIMER_TICK_MS 10u

static volatile uint16_t g_timer_ticks = 0;
static volatile uint8_t  g_timer_flag  = 0;

void timer_initialize()
{
    /* Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
       but the OSCCON register is lock protected. That means you would have to 
       write a specific sequence of numbers to the register OSCCONL. After that 
       the write access to OSCCONL will be enabled for one instruction cycle.
       The function __builtin_write_OSCCONL(val) does the unlocking sequence and
       afterwards writes the value val to that register. (OSCCONL represents the
       lower 8 bits of the register OSCCON)
    */
    __builtin_write_OSCCONL(OSCCONL | 2);

    /* configure Timer1 to use external 32.768 kHz clock
       and generate an interrupt every ~10 ms
    */
    T1CONbits.TON   = 0;        /* stop timer during config     */
    T1CONbits.TCS   = 1;        /* external clock (LPRC/RTC)    */
    T1CONbits.TCKPS = TCKPS_1;  /* prescaler 1:1                */

    TMR1 = 0;

    /* PR1 chosen for ~10 ms:
       32 768 Hz * 0.010 s ≈ 328 counts  -> PR1 ≈ 327
    */
    PR1 = 327;

    /* clear and enable interrupt */
    _T1IF = 0;
    _T1IP = 3;                  /* interrupt priority */
    _T1IE = 1;                  /* enable Timer1 interrupt */

    /* start timer */
    T1CONbits.TON = 1;
}

/* Timer1 interrupt service routine */
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;  /* clear interrupt flag */

    if (g_timer_ticks > 0)
    {
        g_timer_ticks--;
        if (g_timer_ticks == 0)
        {
            g_timer_flag = 1;
        }
    }
}

/* blocking delay using Timer1 "ticks".
   Only multiples of TIMER_TICK_MS are supported. */
static void delay_ms_timer(uint16_t ms)
{
    g_timer_flag  = 0;
    g_timer_ticks = (ms + (TIMER_TICK_MS - 1u)) / TIMER_TICK_MS; /* round up */

    if (g_timer_ticks == 0)
    {
        g_timer_ticks = 1;
    }

    while (!g_timer_flag)
    {
        /* wait */
    }
}

/*
 * main loop
 */

/* approximate DAC codes for the desired voltages.
   You may adjust these based on your Vref / gain configuration. */
#define DAC_CODE_1V     1000u
#define DAC_CODE_2_5V   2500u
#define DAC_CODE_3_5V   3500u

void main_loop()
{
    /* print assignment information */
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: 19");

    while (TRUE)
    {
        /* 1 V output, hold for 500 ms */
        dac_set_raw(DAC_CODE_1V);
        delay_ms_timer(500u);

        /* 2.5 V output, hold for 2000 ms */
        dac_set_raw(DAC_CODE_2_5V);
        delay_ms_timer(2000u);

        /* 3.5 V output, hold for 1000 ms */
        dac_set_raw(DAC_CODE_3_5V);
        delay_ms_timer(1000u);

        /* toggle LED1 after each full sequence */
        LED1_PORT ^= 0x01; // toggle LED 1
        Nop();
    }
}
