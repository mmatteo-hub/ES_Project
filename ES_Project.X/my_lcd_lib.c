/*
 * File:   my_print_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 3 novembre 2022, 11.15
 */

#include "my_lcd_lib.h"
#include <p30F4011.h>
#include <string.h>

void spi_init()
{
    // Function to init the SPI
    SPI1CONbits.MSTEN = 1;  // master mode 
    SPI1CONbits.MODE16 = 0; // 8 bit mode 
    SPI1CONbits.PPRE = 3;   // primary prescaler 
    SPI1CONbits.SPRE = 6;   // secondary prescaler 
    SPI1STATbits.SPIEN = 1; // enable SPI
}

void lcd_clear(short start, short amount)
{
    // Filling an array with spaces
    char chars[amount];
    for(int i=0; i<amount; ++i)
        chars[i] = ' ';
    // Writing the spaces on the LCD
    lcd_write(start, chars);
}

void lcd_write(short start, char chars[])
{
    // Moving the cursor to the correct position
    lcd_move_cursor(start);
 
    // Actually writing on the LCD
    for(int i = 0; chars[i] != '\0'; i++)
    {
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = chars[i];
    }
}

void lcd_move_cursor(short position)
{
    // NB. The arg "position" goes from 0 to 31, after 15 we need to write on the second line
    // The value 0x80 moves the cursor at the beginning of the first line
    // The value 0xC0 moves the cursor at the beginning of the second line
 
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = ((position<16) ? 0x80 : 0xC0) + (position%16);
}