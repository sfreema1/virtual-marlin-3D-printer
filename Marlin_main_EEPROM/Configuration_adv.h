#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

#define AXIS_RELATIVE_MODES {false, false, false, false}

const unsigned int dropsegments = 5; //everything with less than this number of steps will be ignored as move and joined with the next movement

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

#endif // CONFIGURATION_ADV_H
