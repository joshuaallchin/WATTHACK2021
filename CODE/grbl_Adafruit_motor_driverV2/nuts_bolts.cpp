/*
  nuts_bolts.c - Shared functions
*/
/**
 * Part of Grbl interpreter modified to work with Adafruit Motor Driver V2
 * File modified by Catalin Vasiliu 
 * email <vasiliu.catalin.mihai@gmail.com>
 * 
 * 
 * 
 * Original license:***********************************************************
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 * Copyright (c) 2011 Sungeun K. Jeon
 * 
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "nuts_bolts.h"
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

/**
 * Read a floating point value from a string. Line points to the input buffer, char_counter 
 * is the indexer pointing to the current character of the line, while double_ptr is 
 * a pointer to the result variable. Returns true when it succeeds
 * 
 */
int read_double(char *line, uint8_t *char_counter, double *double_ptr)                  
{
  char *start = line + *char_counter;
  char *end;
  
  *double_ptr = strtod(start, &end);
  if(end == start) { 
    return(false); 
  };

  *char_counter = end - line;
  return(true);
}

/**
 *  Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
 *  which only accepts constants in future compiler releases.
 * 
 * @param us
 */
void delay_ms(uint16_t ms) 
{
  while ( ms-- ) { _delay_ms(1); }
}

/**
 * Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
 *  which only accepts constants in future compiler releases.
 * 
 */
void delay_us(uint16_t us) 
{
  while ( us-- ) { _delay_us(1); }
}
