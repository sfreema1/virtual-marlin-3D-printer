#include "Marlin.h"
#include "planner.h"
#include "Configuration.h"
#include "Configuration_adv.h"
#include "ConfigurationStore.h"
#include "pins_arduino.h"
#include "math.h"
#include "stepper.h"
#include "Arduino.h"

/*===== Public Variables =====*/
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply = 100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100};
float volumetric_multiplier[EXTRUDERS] = {1.0};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3] = {0, 0, 0};
bool axis_known_position[3] = {false, false, false};
float zprobe_zoffset;
uint8_t active_extruder = 0;
int fanSpeed = 0;
bool cancel_heatup = false;
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

/*===== Private Variables =====*/
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static char serial_char;
static int buflen = 0;
static int bufindr = 0;
static int bufindw = 0;
static int serial_count = 0;
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static boolean comment_mode = false;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E,
//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000;


/*===== Routines =====*/

void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println("Welcome to Marlin Firmware!");
  Config_RetrieveSettings();
  plan_init();
  st_init();
  Serial.print("Current buffer length: "); Serial.println(buflen);
  Serial.print("Current buffer write index: "); Serial.println(bufindw);
  Serial.print("Current buffer read index: "); Serial.println(bufindr);
  Serial.println("Ready for your input!"); Serial.println("");

}

void loop()
{
  if (buflen < (BUFSIZE - 1))
  {
    get_command();
  }
  if (buflen)
  {
    process_commands();

    Serial.print("Command ");
    Serial.print(cmdbuffer[bufindr]);
    Serial.print(" at buffer read index ");
    Serial.print(bufindr);
    Serial.println(" removed. Command buffer decreased by one.");
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
    Serial.print("Current buffer length: ");
    Serial.println(buflen);
    Serial.print("Current buffer write index: ");
    Serial.println(bufindw);
    Serial.print("Current buffer read index: ");
    Serial.println(bufindr);
    Serial.println("");
  }
}

void get_command()
{
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read();
    if (serial_char == '\n' ||
        serial_char == '\r' ||
        serial_count >= (MAX_CMD_SIZE - 1))
    {
      if (!serial_count)
      {
        Serial.println("Received an empty command line. No command stored. Exiting get_command() process...");
        comment_mode = false;
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0;
      if (!comment_mode)
      {
        Serial.print("Command ");
        Serial.print(cmdbuffer[bufindw]);
        Serial.println(" received and stored. Command buffer length has increased by one.");
        bufindw = (bufindw + 1) % BUFSIZE;
        buflen += 1;
        Serial.print("Current buffer length: ");
        Serial.println(buflen);
        Serial.print("Current buffer write index: ");
        Serial.println(bufindw);
        Serial.print("Current buffer read index: ");
        Serial.println(bufindr);
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if (serial_char == ';') comment_mode = true;
      if (!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

void process_commands()
{
  unsigned long codenum; // throw away variable
  char *starpos = NULL;

  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        Serial.println("G command detected. Executing...");
        get_coordinates(); // For XYZE
        prepare_move();
        return;
        break;
      case 90:
        relative_mode = false;
        Serial.println("Set to absolute positioning.");
        break;
      case 91:
        relative_mode = true;
        Serial.println("Set to relative positioning.");
        break;
      default:
        Serial.print("Error: ");
        Serial.print("Unknown G command: \"");
        Serial.print(cmdbuffer[bufindr]);
        Serial.println("\"");
    }
  }
  else if (code_seen('M'))
  {
  }
  else if (code_seen('T'))
  {
  }
  else
  {
    Serial.print("Error: unknown command ");
    Serial.print(cmdbuffer[bufindr]);
    Serial.println(". Please try another command.");
  }

  ClearToSend();
}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  Serial.println("Clear to send: OK");
}

void get_coordinates()
{
  Serial.print("Retrieved axis destinations of ");
  bool seen[4] = {false, false, false, false};
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    if (code_seen(axis_codes[i]))
    {
      destination[i] = (float )code_value() + (axis_relative_modes[i] || relative_mode) * current_position[i];
      seen[i] = true;
      if (seen[i])
      {
        Serial.print(axis_codes[i]);
        Serial.print(destination[i]);
        Serial.print(" ");
      }
    }
  }
  Serial.println("");
  if (code_seen('F'))
  {
    next_feedrate = code_value();
    if (next_feedrate > 0.0)
    {
      feedrate = next_feedrate;
      Serial.print("Feedrate: ");
      Serial.println(next_feedrate);
      Serial.println("");
    }
  }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if ( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS]))
  {
    Serial.println("No call for X or Y motion detected. Feed multiply will not be used.");
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate / 60, active_extruder);
  }
  else
  {
    Serial.println("Call for X or Y motion detected. Feed multiply will used.");
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate * feedmultiply / 60 / 100.0, active_extruder);
  }
}

void manage_inactivity()
{
  if (buflen < (BUFSIZE - 1)) get_command();

  if ( (millis() - previous_millis_cmd) >  max_inactive_time )
    if (max_inactive_time)
      Serial.println("Max inactive time exceeded. Kill() routine called.");
  if (stepper_inactive_time)
  {
    if ( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if (blocks_queued() == false)
      {
        Serial.println("All axes disabled.");
      }
    }
  }
  // check_axes_activity();
}


void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops)
  {
    Serial.println("Min software endstop settings have been detected. Correcting any prohibited moves...");
    if (target[X_AXIS] < min_pos[X_AXIS])
    {
      target[X_AXIS] = min_pos[X_AXIS];
      Serial.println("X axis destination exceeds its min software endstop. New destination is min X endstop.");
    }
    if (target[Y_AXIS] < min_pos[Y_AXIS])
    {
      target[Y_AXIS] = min_pos[Y_AXIS];
      Serial.println("Y axis destination exceeds its min software endstop. New destination is min Y endstop.");
    }
    if (target[Z_AXIS] < min_pos[Z_AXIS])
    {
      target[Z_AXIS] = min_pos[Z_AXIS];
      Serial.println("Z axis destination exceeds its min software endstop. New destination is min Z endstop.");
    }
  }
  if (max_software_endstops)
  {
    Serial.println("Max software endstop settings have been detected. Correcting any prohibited moves...");
    if (target[X_AXIS] > max_pos[X_AXIS])
    {
      target[X_AXIS] = max_pos[X_AXIS];
      Serial.println("X axis destination exceeds its max software endstop. New destination is max X endstop.");
    }
    if (target[Y_AXIS] > max_pos[Y_AXIS])
    {
      target[Y_AXIS] = max_pos[Y_AXIS];
      Serial.println("Y axis destination exceeds its max software endstop. New destination is max Y endstop.");
    }
    if (target[Z_AXIS] > max_pos[Z_AXIS])
    {
      target[Z_AXIS] = max_pos[Z_AXIS];
      Serial.println("Z axis destination exceeds its max software endstop. New destination is max Z endstop.");
    }
  }
}
