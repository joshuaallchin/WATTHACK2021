/**
 * Part of Grbl interpreter modified to work with Adafruit Motor Driver V2
 * File created by Catalin Vasiliu 
 * email <vasiliu.catalin.mihai@gmail.com>
 */

#ifndef STEPPER_H
#define	STEPPER_H


void shield_begin();
void motors_init();
void spindle_run(int direction, uint32_t rpm);
void spindle_stop();

uint8_t st_onestep(int motor, int direction);

void st_go_home(double *position);

void st_dwell(double seconds);

double st_set_current_position(double x, double y, double z);

uint8_t st_line(double *position, double x, double y, double z, double feed_rate, uint8_t invert_feed_rate);

uint8_t st_arc(double *position, double *target, double *offset, uint8_t axis_0, uint8_t axis_1, 
    uint8_t axis_linear, double feed_rate, uint8_t invert_feed_rate, double radius, uint8_t isclockwise);


void st_machine_park();
//Put all motors to lowest val on axes
void st_go_to_zero(bool zAxis);
void st_calibrate();


#endif	/* STEPPER_H */

