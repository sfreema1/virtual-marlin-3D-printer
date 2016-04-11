#ifndef MARLIN_H
#define MARLIN_H

#include "Configuration.h"
#include "Configuration_adv.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#define  FORCE_INLINE __attribute__((always_inline)) inline

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};
extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float zprobe_zoffset;
extern int fanSpeed;


extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;


/*===== Routines =====*/
void get_command();
void process_commands();
float code_value();
bool code_seen(char code);
void ClearToSend();
void get_coordinates();
void prepare_move();
void clamp_to_software_endstops(float target[3]);

#endif // MARLIN_H
