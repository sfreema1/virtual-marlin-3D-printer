#ifndef LANGUAGE_H
#define LANGUAGE_H

#include "Arduino.h"

#define FORCE_INLINE __attribute__((always_incline)) inline

#define SERIAL_PROTOCOL(x) (Serial.print(x))
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),Serial.write('\n'))
#define SERIAL_PROTOCOLPGM_PAIR(name,value) (serial_echopair_P(PSTR(name),(value)),SERIAL_PROTOCOLPGM(" "))
#define SERIAL_PROTOCOLPAIR_COLON(x,y) (Serial.print(x), (SERIAL_PROTOCOLPGM(": ")), Serial.print(y), SERIAL_PROTOCOLPGM(" "))
#define SERIAL_PROTOCOL_END (Serial.write('\n'))


/*===== Messages =====*/
#define MSG_WELCOME "Welcome to the Virtual 3D TissueBot!!"
#define MSG_STORED_SETTINGS_LOADED "Stored settings loaded"
#define MSG_DEFAULT_SETTINGS_LOADED "Default settings loaded"
#define MSG_INSTRUCTION "Enter Code I1 for information on available commands."
#define MSG_COMMAND_STORED "Command successfully stored."
#define MSG_ERROR_UNKNOWN_COMMAND "Unrecognized command. Please try again."
#define MSG_RETRIEVE_COORDINATES "Retrieving coordinates from command ..."
#define MSG_RETRIEVE_FEEDRATE "Retrieving feedrate from command ..."
#define MSG_RETRIEVE_ARC_COORDINATES "Retrieving arc coordinates from command ..."
#define MSG_PREPARE_MOVE "Preparing move ..."
#define MSG_PREPARE_ARC_MOVE "Preparing arc move ..."
#define MSG_CLAMP_TO_X_MIN "Move clamped to min X endstop ..."
#define MSG_CLAMP_TO_Y_MIN "Move clamped to min Y endstop ..."
#define MSG_CLAMP_TO_Z_MIN "Move clamped to min Z endstop ..."
#define MSG_CLAMP_TO_X_MAX "Move clamped to max X endstop ..."
#define MSG_CLAMP_TO_Y_MAX "Move clamped to max Y endstop ..."
#define MSG_CLAMP_TO_Z_MAX "Move clamped to max Z endstop ..."
#define MSG_NO_FEEDMULTIPLIER "E or Z move only. No feedmultiplier."
#define MSG_USE_FEEDMULTIPLIER "Feedmultiply will be used."
#define MSG_CALCULATE_ABSOLUTE_STEPS "Calculating move in absolute steps for each axis ..."
#define MSG_CALCULATE_RELATIVE_STEPS "Calculating move in relative steps for each axis ..."
#define MSG_CREATE_NEW_BLOCK "Creating new block ..."
#define MSG_MAX_STEP_EVENT "Max steps needed is: "
#define MSG_STEP_COUNT_ERROR "Number of motor steps below threshold. Exiting planning ..."
#define MSG_CALCULATE_DIRECTIONS "Calculating directions ..."
#define MSG_ENABLE_X_AXIS "Enabling X stepper driver ..."
#define MSG_ENABLE_Y_AXIS "Enabling Y stepper driver ..."
#define MSG_ENABLE_Z_AXIS "Enabling Z stepper driver ..."
#define MSG_ENABLE_E_AXIS "Enabling E stepper driver ..."
#define MSG_FEEDRATE_SET_TO_MIN_TRAVELFEEDRATE "Feedrate under threshold. Set to min travel feedrate"
#define MSG_FEEDRATE_SET_TO_MIN_FEEDRATE "Feedrate under threshold. Set to min feedrate"  
#define MSG_CALCULATE_RELATIVE_DISTANCE "Calculating relative distance (in mm) ..."
#define MSG_CALCULATE_TOTAL_EUCLIDEAN_DISTANCE "Calculating total distance to travel in mm (vector sum) ..."

// things to write to serial from Program Memory. Saves 400 to 2k of RAM
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while(ch)
  {
    Serial.write(ch);
    ch = pgm_read_byte(++str);
  }
}

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


#endif // LANGUAGE_H
