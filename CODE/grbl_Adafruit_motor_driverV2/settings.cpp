/**
 * settings.c - eeprom configuration handling 
 * migration from older version of setting not supported because this grbl interpreter
 * have different settings and functionality
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
#include <math.h>
#include "nuts_bolts.h"
#include "settings.h"
#include "eeprom.h"
#include "g_print.h"
#include <avr/pgmspace.h>
#include "protocol.h"
#include "config.h"
#include "stepper_control.h"


settings_t settings;

typedef struct {
  double steps_per_mm[3];
  float rod_step;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
} settings_v1_t;



/**
 * Set settings values to default
 */
void settings_reset() {
    settings.rod_step = DEFAULT_ROD_STEP;
    settings.steps_per_turn[X_AXIS] = DEFAULT_X_STEPS_PER_TURN;
    settings.steps_per_turn[Y_AXIS] = DEFAULT_Y_STEPS_PER_TURN;
    settings.steps_per_turn[Z_AXIS] = DEFAULT_Z_STEPS_PER_TURN;
    settings.steps_per_mm[X_AXIS] = settings.steps_per_turn[X_AXIS] / settings.rod_step;
    settings.steps_per_mm[Y_AXIS] = settings.steps_per_turn[Y_AXIS] / settings.rod_step;
    settings.steps_per_mm[Z_AXIS] = settings.steps_per_turn[Z_AXIS] / settings.rod_step;

    settings.default_feed_rate = DEFAULT_FEEDRATE;
    settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
    settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
    settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
    settings.default_spindle_speed = DEFAULT_SPINDLE_SPEED;

    settings.work_area[X_AXIS] = DEFAULT_WORK_AREA_X_AXIS;
    settings.work_area[Y_AXIS] = DEFAULT_WORK_AREA_Y_AXIS;
    settings.work_area[Z_AXIS] = DEFAULT_WORK_AREA_Z_AXIS;
    
    settings.limit_switch = DEFAULT_LIMIT_SWITCH;
    settings.release_after_move = RELEASE_AFTER_MOVE;
}

/**
 * Print to serial bus settings data and infos
 * Modifying the format of these strings may cause the settings window in 
 * grbl sender to stop working!!!
 */
void settings_dump() {
    printPgmString(PSTR("$0 = ")); 
    printFloat(settings.rod_step);
    printPgmString(PSTR(" (mm/turn ...rod step)\r\n")); 

    printPgmString(PSTR("\r\n $1 = "));
    printInteger(settings.steps_per_turn[X_AXIS]);
    printPgmString(PSTR(" (steps/turn x motor)\r\n"));  

    printPgmString(PSTR("\r\n $2 = ")); 
    printInteger(settings.steps_per_turn[Y_AXIS]);
    printPgmString(PSTR(" (steps/turn y motor)\r\n")); 

    printPgmString(PSTR("\r\n $3 = ")); 
    printInteger(settings.steps_per_turn[Z_AXIS]);
    printPgmString(PSTR(" (steps/turn z motor)\r\n")); 

    printPgmString(PSTR("\r\n $4 = "));
    printFloat(settings.steps_per_mm[X_AXIS]);
    printPgmString(PSTR(" (steps/mm x)\r\n"));  

    printPgmString(PSTR("\r\n $5 = ")); 
    printFloat(settings.steps_per_mm[Y_AXIS]);
    printPgmString(PSTR(" (steps/mm y)\r\n")); 

    printPgmString(PSTR("\r\n $6 = ")); 
    printFloat(settings.steps_per_mm[Z_AXIS]);
    printPgmString(PSTR(" (steps/mm z)\r")); 

    printPgmString(PSTR("\n $7 = ")); 
    printFloat(settings.default_feed_rate);
    printPgmString(PSTR(" (mm/min default feed rate)\r\n"));

    printPgmString(PSTR("\r\n $8 = "));
    printFloat(settings.default_seek_rate);
    printPgmString(PSTR(" (mm/min default seek rate)\r\n")); 

    printPgmString(PSTR("\r\n $9 = ")); 
    printFloat(settings.mm_per_arc_segment);
    printPgmString(PSTR(" (mm/arc segment)\r\n")); 

    printPgmString(PSTR("\r\n $10 = ")); 
    printInteger(settings.invert_mask); 
    printPgmString(PSTR(" (step port invert mask. binary = "));   
    printIntegerInBase(settings.invert_mask, 2);  
    printPgmString(PSTR(")\r\n"));   

    printPgmString(PSTR("\r\n $11 = "));   
    printFloat(settings.default_spindle_speed);
    printPgmString(PSTR(" (default spindle speed in pwm)\r\n"));
    
    printPgmString(PSTR("\r\n $12 = "));   
    printFloat(settings.limit_switch); 
    printPgmString(PSTR(" (limit switch enable/disable, 1/0)\r\n"));
    
    printPgmString(PSTR("\r\n $13 = "));   
    printFloat(settings.release_after_move); 
    printPgmString(PSTR(" (release motors after move enable/disable, 1/0)\r\n"));
    
    //Work area
    printPgmString(PSTR("\r\n $14 = "));   
    printFloat(settings.work_area[X_AXIS]);
    printPgmString(PSTR(" (mm on X axis workin area)\r\n"));

    printPgmString(PSTR("\r\n $15 = "));   
    printFloat(settings.work_area[Y_AXIS]); 
    printPgmString(PSTR(" (mm on Y axis workin area)\r\n"));

    printPgmString(PSTR("\r\n $16 = "));   
    printFloat(settings.work_area[Z_AXIS]); 
    printPgmString(PSTR(" (mm on Z axis workin area)\r\n"));
    
    //Recalibrate
    if(settings.limit_switch == 1){
        printPgmString(PSTR("\r\n $17 = "));   
        printInteger(0); 
        printPgmString(PSTR(" (recalibrate workin area)\r\n"));
    }else{
        printPgmString(PSTR("\r\n $17 = "));   
        printInteger(0); 
        printPgmString(PSTR(" (Recalibrate not available with limit switch disable)\r\n"));
    }

    //Reset...manual
    printPgmString(PSTR("\r\n $18 = "));
    printInteger(0); 
    printPgmString(PSTR(" (1 to reset setings)\r\n"));
    printPgmString(PSTR("\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n"));
}

/**
 * Process the settings line 
 * Parameter lines are on the form '$4=374.3' or '$' to dump current settings
 * 
 * @param char array line
 * @return int status
 */
uint8_t settings_execute_line(char *line) {
    uint8_t char_counter = 1;
    double parameter, value;
    if(line[0] != '$') { 
      return(STATUS_UNSUPPORTED_STATEMENT); 
    }
    
    if(GRBL_VERSION == "0.7") {
        if(line[char_counter] == 0) { 
            settings_dump(); return(STATUS_OK); 
        }
    } else if (GRBL_VERSION == "0.9") {
        if(line[char_counter] == '$') { 
            settings_dump(); return(STATUS_OK); 
        }
    }
    
    if(!read_double(line, &char_counter, &parameter)) {
      return(STATUS_BAD_NUMBER_FORMAT);
    };
    
    if(line[char_counter++] != '=') { 
      return(STATUS_UNSUPPORTED_STATEMENT); 
    }
    
    if(!read_double(line, &char_counter, &value)) {
      return(STATUS_BAD_NUMBER_FORMAT);
    }
    
    if(line[char_counter] != 0) { 
      return(STATUS_UNSUPPORTED_STATEMENT); 
    }
    //Send option and value so save
    settings_store_setting(parameter, value);
    return(STATUS_OK);
}

/**
 * Write all settings to eprom
 */
void write_settings() {
    eeprom_put_char(0, SETTINGS_VERSION);
    memcpy_to_eeprom_with_checksum(1, (unsigned char*)&settings, sizeof(settings_t));
}

/**
 * Read seting from eprom
 * Check settings version and try to migrate to current settings version
 * 
 * @return bool
 */
int read_settings() {
    //Check version-byte of eeprom
    uint8_t version = eeprom_get_char(0);
  
    if (version == SETTINGS_VERSION) {
        //Read settings-record and check checksum
        if(!(memcpy_from_eeprom_with_checksum((unsigned char*)&settings, 1, sizeof(settings_t)))) {
            return(false);
        }
    } else if (version == 1) {
        //Migrate from settings version 1
        if (!(memcpy_from_eeprom_with_checksum((unsigned char*)&settings, 1, sizeof(settings_v1_t)))) {
            return(false);
        }
        write_settings();
    } else if ((version == 2) || (version == 3)) {
        //Migrate from settings version 2 and 3
        if (!(memcpy_from_eeprom_with_checksum((unsigned char*)&settings, 1, sizeof(settings_t)))) {
            return(false);
        }
        write_settings();
    } else {      
        return(false);
    }
    return(true);
}

/**
 * A helper method to set settings from command line
 * 
 * @param parameter
 * @param value
 */
void settings_store_setting(int parameter, double value) {
    switch(parameter) {
        case 0: 
            settings.rod_step = value; 
            settings.steps_per_mm[X_AXIS] = settings.steps_per_turn[X_AXIS] / settings.rod_step;
            settings.steps_per_mm[Y_AXIS] = settings.steps_per_turn[Y_AXIS] / settings.rod_step;
            settings.steps_per_mm[Z_AXIS] = settings.steps_per_turn[Z_AXIS] / settings.rod_step;
            break;
        case 1: case 2: case 3:
            if (value <= 0.0) {
                printPgmString(PSTR("Steps/turn must be > 0.0\r\n"));
                return;
            }
            settings.steps_per_turn[parameter] = value; 
            settings.steps_per_mm[parameter] = settings.steps_per_turn[parameter] / settings.rod_step;
            break;
        case 4: case 5: case 6:
            if (value <= 0.0) {
                printPgmString(PSTR("Steps/mm must be > 0.0\r\n"));
                return;
            }
            settings.steps_per_mm[parameter] = value; 
            break;
        case 7: 
            settings.default_feed_rate = value; 
            break;
        case 8: 
            settings.default_seek_rate = value; 
            break;
        case 9: 
            settings.mm_per_arc_segment = value; 
            break;
        case 10: 
            settings.invert_mask = trunc(value); 
            break;
        case 11:
            settings.default_spindle_speed = value;
            break;
        case 12: 
            settings.limit_switch = value;
            break;
        case 13: 
            settings.release_after_move = value;
            break;
            
        case 14: 
            printPgmString(PSTR("\r\nWork area on X axis\r\n"));
            break;
        case 15: 
            printPgmString(PSTR("\r\nWork area on Y axis\r\n"));
            break;
        case 16: 
            printPgmString(PSTR("\r\nWork area on Z axis\r\n"));
            break;
            
        case 17: 
            if(settings.limit_switch == 1){
                printPgmString(PSTR("\r\nCalibrating... please wait\r\n"));
                st_calibrate();
            }else{
                printPgmString(PSTR("\r\n Not available if no limit switch!!!\r\n"));
            }
            break;
        case 18: 
            if(value == 1){
                settings_reset();
                write_settings();
                settings_dump();
                printPgmString(PSTR("\r\nSetings reseted\r\n"));
            }
            break;
        default: 
            printPgmString(PSTR("\r\nUnknown parameter\r\n"));
            return;
    }
    
    write_settings();
    printPgmString(PSTR("\r\nStored new setting\r\n"));
    
}

/**
 * Initialize the config subsystem
 */
void settings_init() {
    if(read_settings()) {
        printPgmString(PSTR("\r\n'$' to dump current settings\r\n"));
    } else {
        printPgmString(PSTR("\r\nWarning: Failed to read EEPROM settings. Using defaults.\r\n"));
        settings_reset();
        write_settings();
        settings_dump();
    }
}
