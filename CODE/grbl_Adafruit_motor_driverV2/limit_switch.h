/* 
 * File:   limit_switch.h
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

#ifndef LIMIT_SWITCH_H
#define	LIMIT_SWITCH_H

#define X_LIMIT_END_ENABLE 21
#define X_LIMIT_START_ENABLE 31
#define Y_LIMIT_END_ENABLE 41
#define Y_LIMIT_START_ENABLE 51
#define Z_LIMIT_END_ENABLE 61
#define Z_LIMIT_START_ENABLE 71
#define NO_SWTCH_ENABLE 0

#define X_motor 0
#define Y_motor 1
#define Z_motor 2

//check for enabled limit swith
int ls_check(int motor, int dir);

#endif	/* LIMIT_SWITCH_H */

