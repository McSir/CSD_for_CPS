#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>
#include <string.h>
#include <stdio.h>

#include "lcd.h"
#include "led.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF


int main(){
    //Init LCD and LEDs
    lcd_initialize();
    led_init();
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    
    lcd_locate(0, 0);
    lcd_printf("Group Members :");
    
    lcd_locate(0, 1);
    lcd_printf("* Bassel Abdelhaleem");
    
    lcd_locate(0, 2);
    lcd_printf("* Maksim Eibozhenko");
    
    lcd_locate(0, 3);
    lcd_printf("* Vianney Jerry Takou");
    
    lcd_locate(0, 7);
    lcd_printf("Counter = ");
    
    uint8_t counter = 0;
    lcd_locate(10, 7);
    
    
    // Stop
    while(1) {
       lcd_locate(10, 7);
       lcd_printf("%i", counter);
       counter++;
      
       
       LED5_PORT = (counter >> 0) & 1;
       Nop();
       LED4_PORT = (counter >> 1) & 1;
       Nop();
       LED3_PORT = (counter >> 2) & 1;
       Nop();
       LED2_PORT = (counter >> 3) & 1;
       Nop();
       LED1_PORT = (counter >> 4) & 1;
       Nop();
       
       __delay_ms(500);
    }
}

