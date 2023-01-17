/*
 * File:   my_utils.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 14 dicembre 2023, 11.15
 */

#include "my_utils.h"
#include <string.h>
#include <stdarg.h>

short clamp_inplace(float* value, float min, float max)
{
    if(*value < min)
    {
        *value = min;
        return 1;
    }
    if(*value > max)
    {
        *value = max;
        return 1;
    }
    return 0;
}

float clamp(float value, float min, float max)
{
    if(value < min)
        return min;
    if(value > max)
        return max;
    return value;
}

char* float_to_string(float x, char *p, short decimals)
{
    // Going to the end of the buffer
    char *s = p + strlen(p);
    // tens = 10^decimals
    int tens = 1;
    for(int i=0; i<decimals; ++i)
        tens *= 10;

    // The sign of the number
    short sign = x<0 ? -1 : 1;
    // Storing the decimals as integers
    unsigned short decms = (int)(sign*x*tens) % tens;
    // Storing the integer part without decimals
    int units = (int)(sign*x);

    // Converting the decimals
    for(int i=0; i < decimals; i++)
    {
        *--s = (decms % 10) + '0';
        decms /= 10;
    }
    // Adding point after decimals
    *--s = '.';
    // Converting int
    do
    {
        *--s = (units % 10) + '0';
        units /= 10;
    }while (units > 0);
    // Adding sign
    if (x < 0) 
        *--s = '-';

    return s;
}

void strscat(int count, ...) {
    // This is the pointer to the arguments of the function, it is updated
    // later in order to point to the next argument
    va_list argp;
    // This function makes the argp pointer point to the first argument in
    // the stack. It does that by using the last non-vararg agument.
    va_start(argp, count);

    // Actually take the arguments
    char* pointer = va_arg(argp, char*);
    for (int i = 0; i < count -1; ++i)
        pointer = stpcpy(pointer, va_arg(argp, char*));
}