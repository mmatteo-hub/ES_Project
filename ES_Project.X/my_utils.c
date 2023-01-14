/*
 * File:   my_utils.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 14 dicembre 2023, 11.15
 */

#include "my_utils.h"

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
    while (units > 0) 
    {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    // Adding sign
    if (x < 0) 
        *--s = '-';

    return s;
}