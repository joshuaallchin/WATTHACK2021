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


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "config.h"
#include "stepper_control.h"
#include "gcode.h"
#include "protocol.h"
#include "settings.h"
#include "serial.h"


void setup(void)
{
  shield_begin();
  
  serial_init(BAUD_RATE); 
  protocol_init();
  settings_init(); 
  gc_init();
  motors_init();
 


}

void loop() {
    //process the serial protocol
    protocol_process(); 
}
