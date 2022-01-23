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

#include "config.h"
#include <Arduino.h>
#include "limit_switch.h"

/**
 * Check the limit switch according to axis and direction
 * 
 * @param int motor -- axis
 * @param int direction
 * @return int status
 */
int ls_check(int motor, int direction) {
    if (motor == X_motor) {
        if (digitalRead(X_LIMIT_END_PIN) && direction > 0) {
            return X_LIMIT_END_ENABLE;
        }
        if (digitalRead(X_LIMIT_START_PIN) && direction < 0) {
            return X_LIMIT_START_ENABLE;
        }
    }
    if (motor == Y_motor) {
        if (digitalRead(Y_LIMIT_END_PIN) && direction > 0) {
            return Y_LIMIT_END_ENABLE;
        }
        if (digitalRead(Y_LIMIT_START_PIN) && direction < 0) {
            return Y_LIMIT_START_ENABLE;
        }
    }
    if (motor == Z_motor) {
        if (digitalRead(Z_LIMIT_END_PIN) && direction > 0) {
            return Z_LIMIT_END_ENABLE;
        }
        if (digitalRead(Z_LIMIT_START_PIN) && direction < 0) {
            return Z_LIMIT_START_ENABLE;
        }
    }
    return NO_SWTCH_ENABLE;
}
