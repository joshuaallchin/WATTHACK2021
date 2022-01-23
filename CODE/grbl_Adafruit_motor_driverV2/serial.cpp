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

#include <Arduino.h>
#include "serial.h"

/**
 * Initiate the serial bus with the specified boundrate
 * 
 * @param long baudrate
 */
void serial_init(long baudrate) {
  Serial.begin(baudrate);
}

void serial_write(unsigned char data) {
  Serial.write(data);
}

unsigned char serial_read() {
  int result = Serial.read();
  return (result == -1)?SERIAL_NO_DATA : result;
}

