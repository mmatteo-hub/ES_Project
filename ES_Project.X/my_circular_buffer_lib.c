/*
 * File: my_circular_buffer_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 8 novembre 2022, 11.55
 */


#include "xc.h"
#include "my_circular_buffer_lib.h"

void cb_init(volatile circular_buffer *cb, char* arr, int size)
{
    // Init the circular buffer
    cb->container = arr;
    cb->size = size;
    cb->count = 0;
    cb->head = 0;
    cb->tail = 0;
}

void cb_free(volatile circular_buffer *cb)
{
    free(cb->container);
}

int cb_push_back(volatile circular_buffer *cb, char item)
{
    // Add an element to the circular buffer
    if(cb->count == cb->size)
        return -1; // no space to write
    
    // Store the new item
    cb->container[cb->head] = item;
    // Update the buffer head
    cb->head = (cb->head+1) % cb->size;
    cb->count++;
    return 0;
}

int cb_push_back_string(volatile circular_buffer *cb, char* string)
{
    // Computing the length of the string
    int length = 0;
    while(string[length++] != '\0');
    --length;
    
    if(length > (cb->size)-(cb->count))
        return -1;  // Not enough space for the string
    
    // Inserting the whole string
    for(int i=0; i<length; ++i)
    {
        cb->container[cb->head] = string[i];
        cb->head = (cb->head+1) % cb->size;
    }
    cb->count += length;
    
    return 0;
}

int cb_pop_front(volatile circular_buffer *cb, char* item)
{
    // Read an element from the circular buffer
    if(cb->count == 0)
        return 0; // no things to be read
    
    // Pop the item from the buffer tail
    *item = cb->container[cb->tail];
    // Update the tale
    cb->tail = (cb->tail+1) % cb->size;
    cb->count--;
    return 1;
}