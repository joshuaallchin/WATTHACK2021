/**
 * This file initially contained methods that handle  motor control
 * I combined the motor control with motion control
 * 
 * File contains methods for linear movement, arc generator and motor control
 * 
 */

/**
 * Part of Grbl interpreter modified to work with Adafruit Motor Driver V2
 * File created by Catalin Vasiliu 
 * email <vasiliu.catalin.mihai@gmail.com>
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#include <arduino.h>

#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "limit_switch.h"
#include "protocol.h"
#include "gcode.h"

#define X_motor 0
#define Y_motor 1
#define Z_motor 2
#define NUM_AXIES 3

Adafruit_MotorShield YZ_shield = Adafruit_MotorShield(0x60);
Adafruit_MotorShield XSpindle_shield = Adafruit_MotorShield(0x61);

Adafruit_StepperMotor *Motor[3];
Adafruit_DCMotor *spindle = XSpindle_shield.getMotor(SPINDLE_PORT);

static int current_direction;

typedef struct {
    long steps_to_travel;
    long abs_steps_to_travel;
    int dir;
    long step_counter;
} Axis;

Axis axis[3];

/**
 * Change I2C clock to 400 KHz
 * Initialize Adafruit Motor Shields
 */
void shield_begin() {
    //Change the i2c clock to 400KHz
    TWBR = ((F_CPU / 400000l) - 16) / 2;

    YZ_shield.begin();
    XSpindle_shield.begin();

   
}

/**
 * Initialize motors 
 */
void motors_init(){
    Motor[X_motor] = XSpindle_shield.getStepper(settings.steps_per_turn[X_AXIS], 2); //x
    Motor[Y_motor] = YZ_shield.getStepper(settings.steps_per_turn[Y_AXIS], 2); //y
    Motor[Z_motor] = YZ_shield.getStepper(settings.steps_per_turn[Z_AXIS], 1); //z
}

/**
 * Turn ON/OFF spindle motor
 * Set direction and speed
 * RPM not implemented use values for pwm (0-255)
 * 
 * @param int direction
 * @param 32bit int rpm
 */
void spindle_run(int direction, uint32_t rpm) {
    if (direction != current_direction) {
        spindle->setSpeed(rpm);
        if (direction > 0) {
            spindle->run(FORWARD);
        } else if (direction < 0) {
            spindle->run(BACKWARD);
        } else {
            spindle->run(RELEASE);
        }
    }
    current_direction = direction;
}

/**
 * Stop spindle
 */
void spindle_stop() {
    spindle_run(0, 0);
}

/**
 * Check the limit switch according to axis and direction
 * Move the motor one step in the direction...motors are define Motor[]
 * Return status ok or limit switch error
 * 
 * @param int motor
 * @param int direction
 * @return int status
 */
uint8_t st_onestep(int motor, int direction) {
    if(settings.limit_switch == 1){
        uint8_t switchCheck = ls_check(motor, direction);
        if (switchCheck > 0) {
            return switchCheck;
        }
    }
    Motor[motor]->onestep(direction > 0 ? FORWARD : BACKWARD, DOUBLE);
    return STATUS_OK;
}

/**
 * Pause for the @param seconds ... just delay
 * 
 * @param double seconds
 */
void st_dwell(double seconds) {
    delay_ms(seconds * 1000);
}

/**
 * Move the tool from current position to xyz position in a straight line
 * Uses oneStep() function to move motors
 * Calculate the time that took to travel one step on every axes 
 * (eg x1, y1, z0 steps is approx 1.4/step_per_mm and it takes about 8ms to travel)
 * Calculate the time that the action should take according to feed_rate and pause the diference
 * Return status
 * 
 * @param double[3] position  current position (gc.position)
 * @param double x 
 * @param double y
 * @param double z
 * @param double feed_rate / seek_rate
 * @param invert_feed_rate
 * @return int status ...from oneStep / ls_check
 */
uint8_t st_line(double *position, double x, double y, double z, double feed_rate, uint8_t invert_feed_rate) {
    uint8_t status = STATUS_OK;
    //calculate steps to make on each axis
    axis[X_AXIS].steps_to_travel = lround((x - position[X_AXIS]) * settings.steps_per_mm[X_AXIS]);
    axis[Y_AXIS].steps_to_travel = lround((y - position[Y_AXIS]) * settings.steps_per_mm[Y_AXIS]);
    axis[Z_AXIS].steps_to_travel = lround((z - position[Z_AXIS]) * settings.steps_per_mm[Z_AXIS]);

    long i, j, maxsteps = 0;

    for (i = 0; i < NUM_AXIES; ++i) {
        //do not take minus vall
        axis[i].abs_steps_to_travel = abs(axis[i].steps_to_travel);
        //calculate dir
        axis[i].dir = axis[i].steps_to_travel > 0 ? 1 : -1;
        //get the longest distance
        if (maxsteps < axis[i].abs_steps_to_travel) {
            maxsteps = axis[i].abs_steps_to_travel;
        }
        axis[i].step_counter = 0;
    }
    //store here if a step was made on a axis, need to calculate time 
    bool axis_step[3];
    
    for (i = 0; i < maxsteps; ++i) {
        //get the start time
        unsigned long start_step_time = millis();
        for (j = 0; j < NUM_AXIES; ++j) {
            axis[j].step_counter += axis[j].abs_steps_to_travel;
            if (axis[j].step_counter >= maxsteps) {
                axis[j].step_counter -= maxsteps;
                status = st_onestep(j, axis[j].dir);
                //check status and return if limit reach
                if (status != STATUS_OK) {
                    return status;
                }
                axis_step[j] = 1;
            } else {
                axis_step[j] = 0;
            }
        }
        
        //calculate distance traveled
        float distance_traveled_mm = sqrt( 
            ((axis_step[X_AXIS] / settings.steps_per_mm[X_AXIS]) * (axis_step[X_AXIS] / settings.steps_per_mm[X_AXIS])) +
            ((axis_step[Y_AXIS] / settings.steps_per_mm[Y_AXIS]) * (axis_step[Y_AXIS] / settings.steps_per_mm[Y_AXIS])) +
            ((axis_step[Z_AXIS] / settings.steps_per_mm[Z_AXIS]) * (axis_step[Z_AXIS] / settings.steps_per_mm[Z_AXIS]))
        );
        
        //calculate how much time(in milliseconds) should take to travel distance_traveled_mm according to feed rate
        double time_should_take = distance_traveled_mm * 60000 / feed_rate;
        //time that actualy take to travel
        unsigned long step_time = millis() - start_step_time;
        //calculate time to  pause, do not accept val < 0  
        if (time_should_take - step_time > 0) {
            delay_ms(time_should_take - step_time);
        }
    }
    //release motors...see config.h comments for more info
    if(settings.release_after_move == 1){
        for (int j = 0; j < 3; j++) {
            Motor[j]->release();
        }
    }
    return status;
}

/**
 * Move tool to program Zero position
 * 
 * @param double[3] position
 */
void st_go_home(double *position) {
    st_line(position, 0, 0, 0, settings.default_feed_rate, false);
}


float atan3(float dy, float dx) {
    float a = atan2(dy, dx);
    if (a < 0) a = (PI * 2.0) + a;
    return a;
}

/**
 * This method assumes the limits have already been checked.
 * This method assumes the start and end radius match.
 * This method assumes arcs are not >180 degrees (PI radians)
 * 
 * Calculates the length of the arc according to start and end coordinates
 * Brake the arc in segments (segment length defined in settings)
 * Uses the st_line() function to draw each segment according to feed_rate
 * 
 * 
 * @param double[3] position     current position array[x, y, z] also the start of the arc
 * @param double[3] target       end at the arc array[x, y, z]
 * @param double[3] offset       the center of the circle array[x, y, z]
 * @param uint8_t   axis_0       contain 0, 1 or 2 and is the first axis on the selected plan
 *                               the value represent the physical axis (ex 0 is X_AXIS, 1 is Y_AXIS..)
 * @param uint8_t   axis_1       the second axis on the selected plan
 * @param uint8_t   axis_linear  the third axis on the selected plan, this one does not move
 * @param double    feed_rate    in mm/minute or inch/minute
 * @param uint8_t   invert_feed_rate
 * @param double    radius       the radius of the arc calculated in gcode.cpp
 * @param bool      isclockwise  direction to draw the circle
 * @return uint8_t  status
 */
uint8_t st_arc(double *position, double *target, double *offset, uint8_t axis_0, uint8_t axis_1,
        uint8_t axis_linear, double feed_rate, uint8_t invert_feed_rate, double radius, uint8_t isclockwise) {

    uint8_t status = STATUS_OK;

    // find angle of arc (sweep)
    double angle1 = atan3(position[axis_1] - offset[axis_1], position[axis_0] - offset[axis_0]);
    double angle2 = atan3(  target[axis_1] - offset[axis_1], target[axis_0] - offset[axis_0]);
    double theta = angle2 - angle1;

    if (isclockwise == 1 && theta < 0){
        angle2 += 2 * PI;
    }
    else if (isclockwise == 0 && theta > 0) {
        angle1 += 2 * PI;
    }

    theta = angle2 - angle1;

    // get length of arc
    double len = abs(theta) * radius;

    int i, segments = ceil(len * settings.mm_per_arc_segment);
    
    double nx, ny, angle3, scale;
    double arc_target[3];
    
    for (i = 0; i < segments; ++i) {
        // interpolate around the arc
        scale = ((double) i) / ((double) segments);

        angle3 = (theta * scale) + angle1;
        
        //update arc target
        arc_target[axis_0] = offset[axis_0] + cos(angle3) * radius;
        arc_target[axis_1] = offset[axis_1] + sin(angle3) * radius;
        arc_target[axis_linear] = position[axis_linear];
        
        // make a line to that intermediate position
        status = st_line(position, arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], feed_rate, 0);
        //check the status
        if (status != STATUS_OK) {
            return status;
        }
        //update new position
        position[X_AXIS] = arc_target[X_AXIS];
        position[Y_AXIS] = arc_target[Y_AXIS];
        position[Z_AXIS] = arc_target[Z_AXIS];
    }
    //draw the last segment
    status = st_line(position, target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, 0);
    return status;

}

/**
 * put all motors to mechanical zero except the z axis witch is optional
 * be aware of the tool on z axis, it may be lower so it can not reach the limit 
 * switch this will stall the motor and also can cause damages
 * 
 * Works only with limit switch !!!
 * 
 * @param bool zero_Z_axis   
 */
void st_go_to_zero(bool zero_Z_axis) {
    if(settings.limit_switch == 1){
        uint8_t xLimit = 0;
        uint8_t yLimit = 0;
        uint8_t zLimit = 0;

        while (true) {
            if (st_onestep(X_AXIS, -1) == X_LIMIT_START_ENABLE) {
                xLimit = 1;
            }
            if (st_onestep(Y_AXIS, -1) == Y_LIMIT_START_ENABLE) {
                yLimit = 1;
            }
            if (zero_Z_axis) {
                if (st_onestep(Z_AXIS, -1) == Z_LIMIT_START_ENABLE) {
                    zLimit = 1;
                }
                if (xLimit + yLimit + zLimit == 3) {
                    break;
                }
            } else {
                if (xLimit + yLimit == 2) {
                    break;
                }
            }
        }
    }
}

/**
 * Count steps from mechanical zero to mechanical max
 * Set values in settings.work_area[X_AXIS]
 * Values in mm
 * 
 * Works only with limit switch !!!
 */
void st_calibrate() {
    if(settings.limit_switch == 1){
        //Go all to 0
        st_go_to_zero(true);

        uint8_t xLimit = 0;
        uint8_t yLimit = 0;
        uint8_t zLimit = 0;

        double xCounter = 0;
        double yCounter = 0;
        double zCounter = 0;

        while (true) {
            if (st_onestep(X_AXIS, 1) == X_LIMIT_END_ENABLE) {
                xLimit = 1;
            } else {
                xCounter++;
            }
            if (st_onestep(Y_AXIS, 1) == Y_LIMIT_END_ENABLE) {
                yLimit = 1;
            } else {
                yCounter++;
            }
            if (st_onestep(Z_AXIS, 1) == Z_LIMIT_END_ENABLE) {
                zLimit = 1;
            } else {
                zCounter++;
            }
            if (xLimit + yLimit + zLimit == 3) {
                break;
            }
        }
        settings.work_area[X_AXIS] = xCounter / settings.steps_per_mm[X_AXIS];
        settings.work_area[Y_AXIS] = yCounter / settings.steps_per_mm[Y_AXIS];
        settings.work_area[Z_AXIS] = zCounter / settings.steps_per_mm[Z_AXIS];
    }
}

/**
 * Works with M102 and limit swiths
 * Put jogs to a compact position for storage
 * Park it :P
 * In my configuration is easyer to store the machine with 
 * x axis to minimum ad y axis to maximum
 */
void st_machine_park(){
    if(settings.limit_switch == 1){
        uint8_t xLimit = 0;
        uint8_t yLimit = 0;

        while (true) {
            if (st_onestep(X_AXIS, -1) == X_LIMIT_START_ENABLE) {
                xLimit = 1;
            }
            if (st_onestep(Y_AXIS, 1) == Y_LIMIT_END_ENABLE) {
                yLimit = 1;
            }
            if (xLimit + yLimit == 2) {
                break;
            }
        }
    }
}