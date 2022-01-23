/*
  protocol.c - the serial protocol master control unit
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

#include <avr/io.h>
#include "protocol.h"
#include "gcode.h"
#include "serial.h"
#include "print.h"
#include "settings.h"
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>
#include "limit_switch.h"
#include "g_print.h"


#define LINE_BUFFER_SIZE 50

// Line to be executed. Zero-terminated.
static char line[LINE_BUFFER_SIZE]; 
// Last character counter in line variable.
static uint8_t char_counter; 
// Comment/block delete flag for processor to ignore comment characters.
static uint8_t iscomment; 

/**
 * Process status_code and print to serial bus the error or ok
 * 
 * @param int status_code
 */
static void status_message(int status_code) {
    if (status_code == STATUS_OK) {
         printPgmString(PSTR(" <Idle,MPos:30.000,0.000,0.000,WPos:30.000,0.000,0.000>"));
        printPgmString(PSTR("ok\r\n"));
    } else {
        printPgmString(PSTR("error: "));
        switch (status_code) {
            case STATUS_BAD_NUMBER_FORMAT:
                printPgmString(PSTR("Bad number format\r\n"));
                break;
            case STATUS_EXPECTED_COMMAND_LETTER:
                printPgmString(PSTR("Expected command letter\r\n"));
                break;
            case STATUS_UNSUPPORTED_STATEMENT:
                printPgmString(PSTR("Unsupported statement\r\n"));
                break;
            case STATUS_FLOATING_POINT_ERROR:
                printPgmString(PSTR("Floating point error\r\n"));
                break;
            //limit swtch error
            case X_LIMIT_END_ENABLE:
                printPgmString(PSTR("End limit switch on X axis enabled\r\n"));
                break;
            case X_LIMIT_START_ENABLE:
                printPgmString(PSTR("Start limit switch on X axis enabled\r\n"));
                break;
            case Y_LIMIT_END_ENABLE:
                printPgmString(PSTR("End limit switch on Y axis enabled\r\n"));
                break;
            case Y_LIMIT_START_ENABLE:
                printPgmString(PSTR("Start limit switch on Y axis enabled\r\n"));
                break;
            case Z_LIMIT_END_ENABLE:
                printPgmString(PSTR("End limit switch on Z axis enabled\r\n"));
                break;
            case Z_LIMIT_START_ENABLE:
                printPgmString(PSTR("Start limit switch on Z axis enabled\r\n"));
                break;
            default:
                printPgmString(PSTR("Unknown error "));
                printInteger(status_code);
                printPgmString(PSTR("\r\n"));
        }
    }
}

/**
 * Initiate the protocol and send version number to gcode sender
 */
void protocol_init() {
    // Reset line input
    char_counter = 0; 
    iscomment = false;
    printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
    printPgmString(PSTR("\r\n"));
   
}


/**
 * Executes one line of input according to protocol
 * 
 * @param char array line 
 * @return int status message
 */
uint8_t protocol_execute_line(char *line) {
    if (line[0] == '$') {
        //Delegate lines starting with '$' to the settings module
        return (settings_execute_line(line)); 
    } else {
        //Everything else is gcode
        return (gc_execute_line(line)); 
    }
}

/**
 * Process one line of incoming serial data. Remove unneeded characters and capitalize.
 */
void protocol_process() {
    char c;
    while ((c = serial_read()) != SERIAL_NO_DATA) {
        //End of line reached
        if ((c == '\n') || (c == '\r')) { 
            //Line is complete. Then execute!
            if (char_counter > 0) {
                //Terminate string
                line[char_counter] = 0; 
                status_message(protocol_execute_line(line));
            } else {
                //Empty or comment line. Skip block.
                //Send status message for syncing purposes.
                status_message(STATUS_OK); 
            }
            //Reset line buffer index
            char_counter = 0; 
            //Reset comment flag
            iscomment = false; 
        } else {
            if (iscomment) {
                //Throw away all comment characters
                if (c == ')') {
                    //End of comment. Resume line.
                    iscomment = false;
                }
                
            } else {
                if (c <= ' ') {
                    //Throw away whitepace and control characters
                } else if (c == '/') {
                    //Disable block delete and throw away character
                    //To enable block delete, uncomment following line. Will ignore until EOL.
                    //iscomment = true;
                } else if (c == '(') {
                    //Enable comments flag and ignore all characters until ')' or EOL.
                    iscomment = true;
                } else if (char_counter >= LINE_BUFFER_SIZE - 1) {
                    //Throw away any characters beyond the end of the line buffer
                } else if (c >= 'a' && c <= 'z') { 
                    //Upcase lowercase
                    line[char_counter++] = c - 'a' + 'A';
                } else {
                    line[char_counter++] = c;
                }
            }
        }
    }
}
