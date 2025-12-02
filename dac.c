#include "dac.h"

// tristate register
#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13

// port register
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

// analog to digital converter 1 port configuration register
#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

// analog to digital converter 2 port configuration register
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
    
    // set default state: CS=on, SCK=off, SDI=off, LDAC=on
    DAC_CS_PORT   = 1;
    DAC_SCK_PORT  = 0;
    DAC_SDI_PORT  = 0;
    DAC_LDAC_PORT = 1;
}

void dac_convert_milli_volt(uint16_t milliVolt)
{
    uint16_t cmd;
    uint16_t i;
    uint32_t start;

    cmd = 0;
    cmd |= (0u << 13) | (1u << 12);  // GA -> 1x 
                                     // SHDN -> 1 to activate DAC

    cmd |= (milliVolt*1000 & 0x0FFFu); // add data bits

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
}