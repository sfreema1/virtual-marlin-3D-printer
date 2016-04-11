#include "Buffer.h"
#include "Arduino.h"
#include "Language.h"
#include "Configuration.h"
#include "Motion_control.h"
#include "planner.h"

/*===== Variables =====*/
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE]; // command buffer
int buflen = 0; // current length of command buffer
int bufindw = 0; // buffer write index
int bufindr = 0; // buffer read index

char serial_char; // variable for incoming read from serial
int serial_count = 0; // for tracking number of characters received
char *strchr_pointer; // char pointer for parsing commands

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'}; // a char array for storing axis labes
float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0}; // float array for storing destination
float offset[3] = {0.0, 0.0, 0.0};
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool relative_mode = false;
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float feedrate = 1500.0, next_feedrate, saved_feedrate; // feedrates in mm/min
int feedmultiply = 100;
unsigned long previous_millis_cmd = 0;
uint8_t active_extruder = 0;
float volumetric_multiplier[EXTRUDERS] = {1.0};
int extrudemultiply = 100;

/*===== Routines =====*/
float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_command() // Routine to retrieve commands from serial when available
{
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read();
    if (serial_char == '\n' || serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1))
    {
      if (!serial_count) return; // empty line

      cmdbuffer[bufindw][serial_count] = 0;
      bufindw = (bufindw + 1) % BUFSIZE;
      buflen += 1;
      serial_count = 0;
      SERIAL_PROTOCOLLNPGM(MSG_COMMAND_STORED);
    }
    else
    {
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

void get_coordinates()
{
  bool seen[4] = {false, false, false, false};
  SERIAL_PROTOCOLLNPGM(MSG_RETRIEVE_COORDINATES);
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    if (code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode) * current_position[i];
      SERIAL_PROTOCOLPAIR_COLON(axis_codes[i], destination[i]);
    }
    else
    {
      destination[i] = current_position[i];
    }
  }
  SERIAL_PROTOCOL_END;
  if (code_seen('F'))
  {
    SERIAL_PROTOCOLLNPGM(MSG_RETRIEVE_FEEDRATE);
    next_feedrate = code_value();
    SERIAL_PROTOCOLPGM_PAIR("F: ", next_feedrate);
    SERIAL_PROTOCOL_END;
  }
}

void get_arc_coordinates()
{
  get_coordinates();
  SERIAL_PROTOCOLLNPGM(MSG_RETRIEVE_ARC_COORDINATES);
  if (code_seen('I')) offset[0] = code_value();
  else offset[0] = 0.0;
  if (code_seen('J')) offset[1] = code_value();
  else offset[1] = 0.0;
  SERIAL_PROTOCOLPGM_PAIR("I: ", offset[0]);
  SERIAL_PROTOCOLPGM_PAIR("J: ", offset[1]);
  SERIAL_PROTOCOL_END;
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops)
  {
    if (target[X_AXIS] < min_pos[X_AXIS]) {
      target[X_AXIS] = min_pos[X_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_X_MIN);
    }
    if (target[Y_AXIS] < min_pos[Y_AXIS]) {
      target[Y_AXIS] = min_pos[Y_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_Y_MIN);
    }
    if (target[Z_AXIS] < min_pos[Z_AXIS]) {
      target[Z_AXIS] = min_pos[Z_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_Z_MIN);
    }
  }
  if (max_software_endstops)
  {
    if (target[X_AXIS] > max_pos[X_AXIS]) {
      target[X_AXIS] = max_pos[X_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_X_MAX);
    }
    if (target[Y_AXIS] > max_pos[Y_AXIS]) {
      target[Y_AXIS] = max_pos[Y_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_Y_MAX);
    }
    if (target[Z_AXIS] > max_pos[Z_AXIS]) {
      target[Z_AXIS] = max_pos[Z_AXIS];
      SERIAL_PROTOCOLLNPGM(MSG_CLAMP_TO_Z_MAX);
    }
  }
}

void prepare_move()
{
  SERIAL_PROTOCOLLNPGM(MSG_PREPARE_MOVE);
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if ( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS]))
  {
    SERIAL_PROTOCOLLNPGM(MSG_NO_FEEDMULTIPLIER);
    // plan_buffer_line(dest_X (mm), dest_Y (mm), dest_Z (mm), dest_E (mm), feedrate (mm/min) ->/60-> (mm/s), active_extruder)
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate / 60, active_extruder);
  }
  else
  {
    SERIAL_PROTOCOLLNPGM(MSG_USE_FEEDMULTIPLIER);
    // plan_buffer_line(dest_X (mm), dest_Y (mm), dest_Z (mm), dest_E (mm), feedrate (mm/min) ->/60-> (mm/s) ->/100-> amplification of feedrate by feedmutiply, active_extruder)
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate * feedmultiply / 60 / 100.0, active_extruder);
  }
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    current_position[i] = destination[i]; // This sets the destination as the current position so that move planning can occur before actual movement
  }
}

void prepare_arc_move(char isclockwise)
{
  SERIAL_PROTOCOLLNPGM(MSG_PREPARE_ARC_MOVE);
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]);
  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate * feedmultiply / 60 / 100.0, r, isclockwise, active_extruder);
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();

}

void process_commands() // Routine to interpret commands sent over serial
{
  unsigned long codenum; // throwaway variable
  char *starpos = NULL;

  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      case 0:
      case 1: // G1 - Linear motion
        get_coordinates(); // Retrieve coordinate values for X Y Z E axes
        prepare_move();
        return;
      case 2: // G2 - CW Arc motion
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      case 3: // G3 - CCW Arc motion
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      default:
        SERIAL_PROTOCOLLNPGM(MSG_ERROR_UNKNOWN_COMMAND);
        return;
    }
  }
  else
  {
    SERIAL_PROTOCOLLNPGM(MSG_ERROR_UNKNOWN_COMMAND);
  }
}

