#ifndef BUFFER_H
#define BUFFER_H
#include "Configuration.h"

/*===== Variables =====*/
enum AxisEnum {X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E_AXIS = 3};

extern int buflen;
extern int bufindr;
extern int bufindw;
extern bool axis_relative_modes[];
extern float current_position[NUM_AXIS] ;
extern int feedmultiply;
extern uint8_t active_extruder;
extern const char axis_codes[NUM_AXIS];
extern float volumetric_multiplier[EXTRUDERS];
extern int extrudemultiply;

/*===== Routines =====*/
void get_command();
void process_commands();
void get_coordinates();
void get_arc_coordinates();
void clamp_to_software_endstops(float target[3]);
void prepare_move();

#endif // BUFFER_H
