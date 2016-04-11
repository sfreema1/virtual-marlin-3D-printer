#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Arduino.h"

// Communication speed through serial port
#define BAUDRATE 115200

// Maximum block buffer
#define BLOCK_BUFFER_SIZE 16 
// Number of commands that can be stored in the command buffer
#define BUFSIZE 4
// Maximum command length
#define MAX_CMD_SIZE 96

//Number of extruders
#define EXTRUDERS 1

// Pin Setup
#define STEP_PIN 8
#define DIR_PIN 7
#define MS1_PIN 11
#define MS2_PIN 10
#define MS3_PIN 9
#define ENABLE_PIN 12

// Movement Settings
#define NUM_AXIS 4

// Arc interpretation Settings
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25
const unsigned int dropsegments = 5; //everything with less than this number of steps will be ignored as move and joined with the next movement

// Mechanical Settings
#define AXIS_RELATIVE_MODES {false, false, false, false}

// Endstop Settings
#define min_software_endstops true
#define max_software_endstops true
// Travel limits after homing
#define X_MAX_POS 230
#define X_MIN_POS 0
#define Y_MAX_POS 230
#define Y_MIN_POS 0
#define Z_MAX_POS 220
#define Z_MIN_POS 0

// Default Settings
// Leadscrew - Motor Steps per Revolution / Rod Pitch (M8 leadscrew has 1.25 mm pitch)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,2560.0,2560.0}
#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0     // minimum travel feedrate

#endif // CONFIGURATION_H

