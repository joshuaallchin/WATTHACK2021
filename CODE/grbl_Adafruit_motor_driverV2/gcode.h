/**
 * gcode.cpp - rs274/ngc parser.
 * This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the NIST RS274/NGC Interpreter
 * by Kramer, Proctor and Messina.
*/
/**
 * Part of Grbl interpreter modified to work with Adafruit Motor Driver V2
 * File created by Catalin Vasiliu 
 * email <vasiliu.catalin.mihai@gmail.com>
 */

#ifndef gcode_h
#define gcode_h
#include <avr/io.h>

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

#endif
