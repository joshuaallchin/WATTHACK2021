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
 

#include "gcode.h"
#include <string.h>
#include "nuts_bolts.h"
#include <math.h>
#include "settings.h"
#include "stepper_control.h"
#include "errno.h"
#include "protocol.h"
#include <Arduino.h>

#define MM_PER_INCH (25.4)

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2
#define NEXT_ACTION_SET_COORDINATE_OFFSET 3
#define NEXT_ACTION_MACHINE_ZERO 4
#define NEXT_ACTION_MACHINE_ZERO_EXCEPT_Z 5
#define NEXT_ACTION_MACHINE_PARK 6

#define MOTION_MODE_SEEK 0 // G0 
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

typedef struct {
    uint8_t status_code;
    uint8_t motion_mode; /* {G0, G1, G2, G3, G80} */
    uint8_t inverse_feed_rate_mode; /* G93, G94 */
    uint8_t inches_mode; /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
    uint8_t absolute_mode; /* 0 = relative motion, 1 = absolute motion {G90, G91} */
    uint8_t program_flow;
    double feed_rate, seek_rate; /* Millimeters/second */
    double position[3]; /* Where the interpreter considers the tool to be at this point in the code */
    uint8_t tool;
    int16_t spindle_speed; /* RPM/100 */
    int8_t spindle_direction;
    uint8_t plane_axis_0,
    plane_axis_1,
    plane_axis_2; // The axes of the selected plane  
} parser_state_t;

static parser_state_t gc;

#define FAIL(status) gc.status_code = status;

static int next_statement(char *letter, double *double_ptr, char *line, uint8_t *char_counter);

static void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2) {
    gc.plane_axis_0 = axis_0;
    gc.plane_axis_1 = axis_1;
    gc.plane_axis_2 = axis_2;
}

/**
 * 
 */
void gc_init() {
    memset(&gc, 0, sizeof (gc));
    gc.feed_rate = settings.default_feed_rate;
    gc.seek_rate = settings.default_seek_rate;
    select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    gc.absolute_mode = true;
    gc.spindle_speed = settings.default_spindle_speed;
}

/**
 * Convert value from mm to inch if gc.inches_mode is active
 * 
 * @param double value
 * @return float converted value
 */
static float to_millimeters(double value) {
    return (gc.inches_mode ? (value * MM_PER_INCH) : value);
}

/**
 * Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
 * characters and signed floating point values (no whitespace). Comments and block delete
 * 
 * @param char array
 * @return uint8_t status code
 */
uint8_t gc_execute_line(char *line) {
    uint8_t char_counter = 0;
    char letter;
    double value;
    
    double unit_converted_value;
    // negative inverse_feed_rate means no inverse_feed_rate specified
    double inverse_feed_rate = -1; 
   
    uint8_t radius_mode = false;
    /* 1 = absolute motion for this block only {G53} */
    uint8_t absolute_override = false; 
    /* The action that will be taken by the parsed line */
    uint8_t next_action = NEXT_ACTION_DEFAULT; 

    double target[3], offset[3];
    
    double p = 0, r = 0;
    int int_value;

    gc.status_code = STATUS_OK;

    //Commands
    while (next_statement(&letter, &value, line, &char_counter)) {
        int_value = trunc(value);
        switch (letter) {
            case 'G':
                switch (int_value) {
                    case 0: gc.motion_mode = MOTION_MODE_SEEK;
                        break;
                    case 1: gc.motion_mode = MOTION_MODE_LINEAR;
                        break;
                    case 2: gc.motion_mode = MOTION_MODE_CW_ARC;
                        break;
                    case 3: gc.motion_mode = MOTION_MODE_CCW_ARC;
                        break;
                    case 4: next_action = NEXT_ACTION_DWELL;
                        break;
                    case 17: select_plane(X_AXIS, Y_AXIS, Z_AXIS);
                        break;
                    case 18: select_plane(X_AXIS, Z_AXIS, Y_AXIS);
                        break;
                    case 19: select_plane(Y_AXIS, Z_AXIS, X_AXIS);
                        break;
                    case 20: gc.inches_mode = true;
                        break;
                    case 21: gc.inches_mode = false;
                        break;
                    case 28: case 30: next_action = NEXT_ACTION_GO_HOME;
                        break;
                    case 53: absolute_override = true;
                        break;
                    case 80: gc.motion_mode = MOTION_MODE_CANCEL;
                        break;
                    case 90: gc.absolute_mode = true;
                        break;
                    case 91: gc.absolute_mode = false;
                        break;
                    case 92: next_action = NEXT_ACTION_SET_COORDINATE_OFFSET;
                        break;
                    case 93: gc.inverse_feed_rate_mode = true;
                        break;
                    case 94: gc.inverse_feed_rate_mode = false;
                        break;
                    default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
                }
                break;

            case 'M':
                switch (int_value) {
                    case 0: case 1: gc.program_flow = PROGRAM_FLOW_PAUSED;
                        break;
                    case 2: case 30: case 60: gc.program_flow = PROGRAM_FLOW_COMPLETED;
                        break;
                    case 3: gc.spindle_direction = 1;
                        break;
                    case 4: gc.spindle_direction = -1;
                        break;
                    case 5: gc.spindle_direction = 0;
                        break;
                    //non standard commands implemented to ease the manual controll
                    //available only with limit switch
                    case 100: next_action = NEXT_ACTION_MACHINE_ZERO; 
                        break;
                    //available only with limit switch
                    case 101: next_action = NEXT_ACTION_MACHINE_ZERO_EXCEPT_Z; 
                        break;
                    case 102: next_action = NEXT_ACTION_MACHINE_PARK; 
                        break;
                    default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
                }
                break;
                
            case 'T': gc.tool = trunc(value);
                break;
        }
        if (gc.status_code) {
            break;
        }
    }

    // If there were any errors parsing this line, return the bad news
    if (gc.status_code) {
        return (gc.status_code);
    }

    char_counter = 0;
    clear_vector_double(target);
    clear_vector_double(offset);
    
    //i.e. target = gc.position
    memcpy(target, gc.position, sizeof (target)); 

    //Parameters
    while (next_statement(&letter, &value, line, &char_counter)) {
        int_value = trunc(value);
        unit_converted_value = to_millimeters(value);
        switch (letter) {
            case 'F':
                // Must be greater than zero
                if (unit_converted_value <= 0) {
                    FAIL(STATUS_BAD_NUMBER_FORMAT);
                } 
                if (gc.inverse_feed_rate_mode) {
                    // seconds per motion for this motion only
                    inverse_feed_rate = unit_converted_value; 
                } else {
                    if (gc.motion_mode == MOTION_MODE_SEEK) {
                        // millimeters or inch per minute
                        gc.seek_rate = unit_converted_value; 
                    } else {
                        gc.feed_rate = unit_converted_value;
                    }
                }
                break;
            case 'I': case 'J': case 'K': offset[letter - 'I'] = unit_converted_value;
                break;
            case 'P': p = value;
                break;
            case 'R': r = unit_converted_value;
                radius_mode = true;
                break;
            case 'S': gc.spindle_speed = value;
                break;
            case 'X': case 'Y': case 'Z':
                if (gc.absolute_mode || absolute_override) {
                    target[letter - 'X'] = unit_converted_value;
                } else {
                    target[letter - 'X'] += unit_converted_value;
                }
                break;
        }
    }

    // If there were any errors parsing this line, return the bad news
    if (gc.status_code) {
        return (gc.status_code);
    }
   
    // Update spindle state
    spindle_run(gc.spindle_direction, gc.spindle_speed);

    // Perform any physical actions
    switch (next_action) {
        case NEXT_ACTION_GO_HOME:
            st_go_home(gc.position);
//            clear_vector_double(target);
            break;
        case NEXT_ACTION_DWELL:
            st_dwell(p);
            break;
        case NEXT_ACTION_SET_COORDINATE_OFFSET:
            gc.position[X_AXIS] = target[X_AXIS];
            gc.position[Y_AXIS] = target[Y_AXIS];
            gc.position[Z_AXIS] = target[Z_AXIS];
            clear_vector_double(target);
            break;
        case NEXT_ACTION_MACHINE_ZERO:
            st_go_to_zero(true);
            gc.position[X_AXIS] = 0;
            gc.position[Y_AXIS] = 0;
            gc.position[Z_AXIS] = 0;
            break;
        case NEXT_ACTION_MACHINE_ZERO_EXCEPT_Z:
            st_go_to_zero(false);
            gc.position[X_AXIS] = 0;
            gc.position[Y_AXIS] = 0;
//            gc.position[Z_AXIS] = 0;
            break;
        case NEXT_ACTION_MACHINE_PARK :
            st_machine_park();
            break;
        case NEXT_ACTION_DEFAULT:
            switch (gc.motion_mode) {
                case MOTION_MODE_CANCEL: break;
                case MOTION_MODE_SEEK:
                    gc.status_code = st_line(gc.position, target[X_AXIS], 
                        target[Y_AXIS], target[Z_AXIS], gc.seek_rate, false);
                    break;
                case MOTION_MODE_LINEAR:
                    gc.status_code = st_line(gc.position, target[X_AXIS], target[Y_AXIS], target[Z_AXIS],
                        (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);
                    break;
                case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
                    if (radius_mode) {
                            // Calculate the change in position along each selected axis
                            double x = target[gc.plane_axis_0] - gc.position[gc.plane_axis_0];
                            double y = target[gc.plane_axis_1] - gc.position[gc.plane_axis_1];

                            clear_vector(offset);
                            double h_x2_div_d = -sqrt(4 * r * r - x * x - y * y) / hypot(x, y); // == -(h * 2 / d)
                            // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
                            // real CNC, and thus - for practical reasons - we will terminate promptly:
                            if (isnan(h_x2_div_d)) {
                                FAIL(STATUS_FLOATING_POINT_ERROR);
                                return (gc.status_code);
                            }
                            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                            if (gc.motion_mode == MOTION_MODE_CCW_ARC) {
                                h_x2_div_d = -h_x2_div_d;
                            }
                            if (r < 0) {
                                h_x2_div_d = -h_x2_div_d;
                                r = -r; // Finished with r. Set to positive for mc_arc
                            }
                            // Complete the operation by calculating the actual center of the arc
                            offset[gc.plane_axis_0] = 0.5 * (x - (y * h_x2_div_d));
                            offset[gc.plane_axis_1] = 0.5 * (y + (x * h_x2_div_d));

                    } else { 
                        // Offset mode specific computations
                        // Compute arc radius for mc_arc
                        r = hypot(offset[gc.plane_axis_0], offset[gc.plane_axis_1]); 
                    }

                    // Set clockwise/counter-clockwise sign for st_arc computations
                    uint8_t isclockwise = false;
                    if (gc.motion_mode == MOTION_MODE_CW_ARC) {
                        isclockwise = true;
                    }

                    // Trace the arc
                    gc.status_code = st_arc(gc.position, target, offset, gc.plane_axis_0, gc.plane_axis_1, gc.plane_axis_2,
                            (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode,
                            r, isclockwise);

                    break;
            }
    }
    //gc.position[] = target[];
    memcpy(gc.position, target, sizeof (double)*3); 
    clear_vector_double(target);
    clear_vector_double(offset);

    return (gc.status_code);
}

/**
 * Parses the next statement and leaves the counter on the first character following
 * the statement. Returns 1 if there was a statements, 0 if end of string was reached
 * or there was an error (check state.status_code).
 * 
 * @param char letter
 * @param double double_ptr
 * @param char array line
 * @param uint8_t char_counter
 * @return int
 */
static int next_statement(char *letter, double *double_ptr, char *line, uint8_t *char_counter) {
    if (line[*char_counter] == 0) {
        return (0); // No more statements
    }

    *letter = line[*char_counter];
    if ((*letter < 'A') || (*letter > 'Z')) {
        FAIL(STATUS_EXPECTED_COMMAND_LETTER);
        return (0);
    }
    (*char_counter)++;
    if (!read_double(line, char_counter, double_ptr)) {
        FAIL(STATUS_BAD_NUMBER_FORMAT);
        return (0);
    };
    return (1);
}

