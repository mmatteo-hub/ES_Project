#include "parser.h"

int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}

int extract_integer(char** pointer_to_payload) 
{
    // Obtaining the payload from its pointer
    char* payload = *pointer_to_payload;
    int number = 0, sign = 1;
    // Getting the sign of the number if present
    if (payload[0] == '-') 
    {
        sign = -1;
        payload++;
    }
    else if (payload[0] == '+') 
    {
        sign = 1;
        payload++;
    }
    // Getting the integer
    while (payload[0] != ',' && payload[0] != '\0') 
    {
        number *= 10;               // multiply the current number by 10;
        number += payload[0] - '0'; // converting character to decimal number
        payload++;
    }
    
    // Modifying the pointer to advance the payload
    *pointer_to_payload = payload;
    if(payload[0] == ',')
        (*pointer_to_payload)++;
        
    return sign*number;
}

float extract_float(char** pointer_to_payload) 
{
    // Obtaining the payload from its pointer
    char* payload = *pointer_to_payload;
    float number = 0, sign = 1;
    // Getting the sign of the number if present
    if (payload[0] == '-') 
    {
        sign = -1;
        payload++;
    }
    else if (payload[0] == '+') 
    {
        sign = 1;
        payload++;
    }
    // Getting the integer
    while (payload[0] != ',' && payload[0] != '.' && payload[0] != '\0') 
    {
        number *= 10;               // multiply the current number by 10;
        number += payload[0] - '0'; // converting character to decimal number
        payload++;
    }
    // Getting the decimals
    if(payload[0] == '.')
    {
        payload++;
        float divider = 1.0;
        while (payload[0] != ',' && payload[0] != '\0') 
        {
            divider *= 10;
            number += (payload[0]-'0') / divider;
            payload++;
        }
    }
    
    // Modifying the pointer to advance the payload
    *pointer_to_payload = payload;
    if(payload[0] == ',')
        (*pointer_to_payload)++;
        
    return sign*number;
}

