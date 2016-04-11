#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "Arduino.h"

/*===== Routines =====*/

// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
void mc_arc(float *position, float *target, float *offset, unsigned char axis_0, unsigned char axis_1, unsigned char axis_linear, float feed_rate, float radius, unsigned char isclockwise, uint8_t extruder);


#endif // MOTION_CONTROL_H
