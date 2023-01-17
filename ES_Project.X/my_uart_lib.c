/*
 * File:   my_uart_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 14 dicembre 2023, 11.15
 */

#include "my_uart_lib.h"
#include <p30F4011.h>

// The value of FCY divided by 16 ((7372800 / 4) / 16)
#define FCY_16 115200

void _uart_in_buffer_fill();
void _handle_uart_overflow();
void _uart_out_buffer_purge();

// The buffer used for reading from UART
volatile circular_buffer *_in_buffer;
// The buffer used for writing to UART
volatile circular_buffer *_out_buffer;


void uart_init(int baudrate, volatile circular_buffer *in_buffer, volatile circular_buffer *out_buffer)
{
    // Function to init the UART, it will trigger the UART buffer full interrupt 
    // when it is full at 3/4. 
    U2BRG = (FCY_16/baudrate)-1;    // (7372800 / 4) / (16 * baudrate) - 1
    U2MODEbits.UARTEN = 1;          // enable UART 
    U2STAbits.UTXEN = 1;            // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10;       // set the receiver interrupt when buffer is 3/4 full
    U2STAbits.UTXISEL = 1;          // set the transmitter interrupt when buffer is empty
    IEC1bits.U2RXIE = 1;            // enable UART receiver interrupt
    IEC1bits.U2TXIE = 1;            // enable UART transmitter interrupt
    // Storing the buffers for using them later
    _in_buffer = in_buffer;
    _out_buffer = out_buffer;
}


void uart_main_loop() {
    // Temporarely disable the UART interrupt to read data
    // This does not cause problems if data arrives now since we are empting the buffer
    IEC1bits.U2RXIE = 0;
    // Handle the reading of the buffer in case there is something to read
    _uart_in_buffer_fill();
    // Re-enable UART interrupt again
    IEC1bits.U2RXIE = 1;
    // Check if there was an overflow in the UART buffer
    _handle_uart_overflow();
}


// This is triggered when the receiver UART buffer is considered full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Handle the reading from the buffer
    _uart_in_buffer_fill();
    // Reset the interrupt flag
    IFS1bits.U2RXIF = 0;
}


// This is triggered when the transmitter UART buffer becomes empty
void __attribute__((__interrupt__, __auto_psv__)) _U2TXInterrupt()
{
    // Handle the writing on the buffer
    _uart_out_buffer_purge();
    // Reset the interrupt flag
    IFS1bits.U2TXIF = 0;
}


void uart_send(char* message) 
{
    // Temporarely disabling the interrupt for the "UART transmission buffer empty" event.
    // This is done because were are now filling it, and we want to prevent concurrent
    // calls to the _uart_out_buffer_purge function (which is also called by the interrupt).
    IEC1bits.U2TXIE = 0;
    // The message needs to be stored on the buffer in case it is not possible to
    // write the whole string on the UART because it is occupied.
    if (cb_push_back_string(_out_buffer, message)== -1)
        return;
    _uart_out_buffer_purge();
    // Re-enabling the interrupt
    IEC1bits.U2TXIE = 1;
}


void _uart_in_buffer_fill()
{
    // Check if there is something to read from UART and then fill the input buffer
    // with that data.
    while(U2STAbits.URXDA == 1)
        cb_push_back(_in_buffer, U2RXREG);
}


void _uart_out_buffer_purge()
{
    // Trasmit data if the UART transmission buffer is not full and 
    // there is actually something to transmit in the output buffer.
    // If not all the data is transmitted (UART buffer is full) this 
    // function will be called again when the UART buffer empties.
    char character;
    while (U2STAbits.UTXBF == 0)
    {
        if(!cb_pop_front(_out_buffer, &character))
            break;
        U2TXREG = character;
    }
}


void _handle_uart_overflow()
{
    // Overflow did not occur, do nothing
    if(U2STAbits.OERR == 0)
        return;
    // The overflow is handled by disgarding all the data in buffer since
    // when a byte is lost, the rest is useless.
    // This is done by clearing the UART overflow flag.
    U2STAbits.OERR = 0;
}