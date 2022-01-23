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

#ifndef settings_h
#define settings_h

#include <math.h>
#include <inttypes.h>

#define GRBL_VERSION "0.9"

//Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
//When firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 4

//Current global settings (persisted in EEPROM from byte 1 onwards)
typedef struct {
    float rod_step;
    int steps_per_turn[3];
    double steps_per_mm[3];
    double default_feed_rate;
    double default_seek_rate;
    uint8_t invert_mask;
    double mm_per_arc_segment;
    int default_spindle_speed;
    double work_area[3];
    bool limit_switch;
    bool release_after_move;
} settings_t;

extern settings_t settings;

//Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

//Print current settings
void settings_dump();

//Handle settings command
uint8_t settings_execute_line(char *line);

//A helper method to set new settings from command line
void settings_store_setting(int parameter, double value);

#endif
