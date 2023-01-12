/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File: my_circular_buffer_lib.h
 * Author: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MY_CIRCULAR_BUFFER_LIB_H
#define MY_CIRCULAR_BUFFER_LIB_H

#include <xc.h> // include processor files - each processor file is guarded.

#include <stdlib.h>

typedef struct circular_buffer {
    char* container;      // the container of the items
    int size;             // the size of the container
    int count;            // number of items in the buffer
    int head;             // index to the first element
    int tail;             // index to the last element
} circular_buffer;

void cb_init(volatile circular_buffer *cb, char *arr, int size);
void cb_free(volatile circular_buffer *cb);
int cb_push_back(volatile circular_buffer *cb, char item);
int cb_push_back_string(volatile circular_buffer *cb, char* string);
int cb_pop_front(volatile circular_buffer *cb, char* item);

#endif

