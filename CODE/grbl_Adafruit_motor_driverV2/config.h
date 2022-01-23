/*
 * config.h - compile time configuration
 */
/**
 * Part of Grbl interpreter modified to work with Adafruit Motor Driver V2
 * File created by Catalin Vasiliu 
 * email <vasiliu.catalin.mihai@gmail.com>
 */

#ifndef config_h
#define config_h

    #define BAUD_RATE 115200
    #define SPINDLE_PORT 1    //on adafruit motor shield
    
    
    //limit swtch pins
    #define X_LIMIT_START_PIN 2
    #define X_LIMIT_END_PIN 3
    #define Y_LIMIT_START_PIN 4
    #define Y_LIMIT_END_PIN 5
    #define Z_LIMIT_START_PIN 6
    #define Z_LIMIT_END_PIN 7

    #define DEFAULT_WORK_AREA_X_AXIS 474.956
    #define DEFAULT_WORK_AREA_Y_AXIS 375.208
    #define DEFAULT_WORK_AREA_Z_AXIS 277.031

    

    /*
     * Number of arc generation iterations by small angle approximation before 
     * exact arc trajectory correction. This parameter maybe decreased if there 
     * are issues with the accuracy of the arc generations. In general, the 
     * default value is more than enough for the intended CNC applications of 
     * grbl, and should be on the order or greater than the size of the buffer 
     * to help with the computational efficiency of generating arcs.
    */
    #define N_ARC_CORRECTION 25 // Integer (1-255)


    //Default settings (used when resetting eeprom-settings)
    #define DEFAULT_ROD_STEP 1.25  // mm per turn
    #define DEFAULT_X_STEPS_PER_TURN 48
    #define DEFAULT_Y_STEPS_PER_TURN 48
    #define DEFAULT_Z_STEPS_PER_TURN 48

    #define DEFAULT_MM_PER_ARC_SEGMENT 0.1
    #define DEFAULT_RAPID_FEEDRATE 500.0 // mm/min
    #define DEFAULT_FEEDRATE 500.0
    #define DEFAULT_STEPPING_INVERT_MASK 1
    #define DEFAULT_SPINDLE_SPEED 255;
    
    /**
     * default to zero for safety
     */
    #define DEFAULT_LIMIT_SWITCH 0;
    /*
     * Some times motor can get very hot because the driver keep one coil 
     * energized to keep the motor blocked, so if you use rod transmission you 
     * don't need that holding torque and you can release the motors so they 
     * will be used only to move
     * It can be changed from settings
     */
    #define RELEASE_AFTER_MOVE 1;


#endif

