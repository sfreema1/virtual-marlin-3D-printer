#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This determines the communication speed of the printer
#define BAUDRATE 115200

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,400.0,500.0}
#define DEFAULT_MAX_FEEDRATE          {500, 500, 5, 25}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,500}    // X, Y, Z, E maximum start speed for accelerated moves

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E

#define EXTRUDERS 1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS. (Kyle changed from "true" to "false")
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below. (Kyle changed from "true" to "false")

// Travel limits after homing
#define X_MAX_POS 230
#define X_MIN_POS 0
#define Y_MAX_POS 230
#define Y_MIN_POS 0
#define Z_MAX_POS 220
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

#endif // CONFIGURATION_H
