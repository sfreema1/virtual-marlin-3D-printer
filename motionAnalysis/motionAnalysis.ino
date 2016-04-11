/*===== Compiler Definitions =====*/
/*-----Communication-----*/
#define BAUDRATE 115200 // Baudrate for communication between computer and printer
#define  FORCE_INLINE __attribute__((always_inline)) inline
#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START
/*-----Buffer Dimensions-----*/
#define MAX_CMD_SIZE 96 // Max space for one command to be stored
#define BUFSIZE 4 // Max number of commands that can be stored
/*-----Printer Settings-----*/
#define NUM_AXIS 4
#define AXIS_RELATIVE_MODES {false, false, false, false}
#define X_MAX_POS 230
#define X_MIN_POS 0
#define Y_MAX_POS 230
#define Y_MIN_POS 0
#define Z_MAX_POS 220
#define Z_MIN_POS 0
#define min_software_endstops true
#define max_software_endstops true
/*-----Planner Settings-----*/
#define BLOCK_BUFFER_SIZE 16
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)
/*=====Public Variables=====*/
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
enum AxisEnum {X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E_AXIS = 3};
float min_pos[3] = {X_MIN_POS, Y_MIN_POS, Z_MIN_POS};
float max_pos[3] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};
extern int feedmultiply = 100; // 100->1 200->2
extern uint8_t active_extruder = 0;
extern float axis_steps_per_unit[4] = {80.0, 80.0, 400.0, 500.0};
float volumetric_multiplier[1] = {1.0};
// The current position of the tool in absolute steps
long position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment
int extrudemultiply = 100; //100->1 200->2
const unsigned int dropsegments = 5;
int fanSpeed = 0;
float mintravelfeedrate = 0;
float minimumfeedrate = 0;
float minsegmenttime = 20000;
float max_feedrate[4] = {500, 500, 5, 25};
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS] = {0, 0, 0, 0};
float retract_acceleration = 3000;
float acceleration = 3000;
extern unsigned long max_acceleration_units_per_sq_second[4] = {9000, 9000, 100, 500};
float max_xy_jerk = 20.0; //speed than can be stopped at once, if i understand correctly.
float max_z_jerk = 0.4;
float max_e_jerk = 5.0;


/*=====Private Variables =====*/
/*-----Buffer Variables-----*/
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE]; // command buffer
static int buflen = 0; // current number of commands stored in the buffer
static int bufindr = 0; // buffer index for reading
static int bufindw = 0; // buffer index for writing
static char serial_char; // received char from serial
static int serial_count = 0; // current tally of chars in current command
static boolean comment_mode = false; // tracks whether current char is part of a comment
static char* strchr_pointer;
/*-----Motion Variables-----*/
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
static bool relative_mode = false; // Determines absolute or relative coordinates
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
/*-----Inactivity Shutdown Variables-----*/
static unsigned long previous_millis_cmd = 0;
/*-----Planner Variables-----*/
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder

  // Fields used by the motion planner to manage acceleration
  //  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  volatile char busy;
} block_t;

static int8_t next_block_index(int8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE)
  {
    block_index = 0;
  }
  return (block_index);
}
volatile unsigned char block_buffer_head; // Index of the next block to be pushed
volatile unsigned char block_buffer_tail; // Index of the block to process now
block_t block_buffer[BLOCK_BUFFER_SIZE];


void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println("Welcome to Marlin Firmware!");
  Serial.println("Please enter a G1 command and see what happens.");
  Serial.println("Do not use line number or checksums.");
  Serial.print("Current buffer length line: ");
  Serial.println(buflen);
  Serial.print("Current buffer write line: ");
  Serial.println(bufindw);
  Serial.print("Current buffer read line: ");
  Serial.println(bufindr);
  Serial.println("");

  reset_acceleration_rates();

}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates()
{
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
    Serial.println(axis_steps_per_sqr_second[i]);
  }
}

void loop()
{
  if (buflen < (BUFSIZE - 1))
  {
    get_command();
  }

  if (buflen)
  {
    process_command();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
    Serial.println("Command processed. Buffer length has decreased by one.");
    Serial.print("Current buffer length: ");
    Serial.println(buflen);
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
        (serial_char == ':' && comment_mode == false) ||
        serial_count >= (MAX_CMD_SIZE - 1))
    {
      if (!serial_count)
      {
        comment_mode = false;
        return;
      }

      cmdbuffer[bufindw][serial_count] = 0;

      if (!comment_mode)
      {
        Serial.print("Command received: ");
        Serial.println(cmdbuffer[bufindw]);
        bufindw = (bufindw + 1) % BUFSIZE;
        buflen += 1;
        Serial.println("Command stored. Buffer length has increased by one.");
        Serial.print("Current buffer length: ");
        Serial.println(buflen);
        Serial.print("Current buffer write index: ");
        Serial.println(bufindw);

      }
      serial_count = 0;
    }
    else
    {
      if (serial_char == ';') comment_mode = true;
      if (!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

void process_command()
{
  unsigned long codenum; // throw away variable
  char *starpos = NULL;
  if (code_seen('G'))
  {
    Serial.println("G command detected. Executing...");
    switch ((int)code_value())
    {
      case 0:
      case 1:
        get_coordinates(); //for XYZE
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
  else
  {
    Serial.print("Error: ");
    Serial.print("Unknown command: \"");
    Serial.print(cmdbuffer[bufindr]);
    Serial.println("\"");

  }

}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);
}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
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
}

void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();

  //Do not use feed multiply for E or Z only moves
  if ( (current_position[X_AXIS] == destination[X_AXIS]) && (current_position[Y_AXIS] == destination[Y_AXIS]))
  {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate / 60, active_extruder);
  }
  else
  {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate * feedmultiply / 60 / 100.0, active_extruder);
  }

  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    current_position[i] = destination[i];
  }
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



void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder) //originally found in planner.cpp
{
  int next_buffer_head = next_block_index(block_buffer_head);
  Serial.println(next_buffer_head);
  while (block_buffer_tail == next_buffer_head)
  {
    Serial.println("Block buffer is currently full. In the meantime Marlin will continue with housekeeping...");
  }

  long target[4];
  target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

  for (int8_t i = 0; i < 4; i++)
  {
    if (target[i])
    {
      Serial.print("Number of steps: ");
      Serial.print(axis_codes[i]);
      Serial.print(" ");
      Serial.println(target[i]);
    }
  }

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  // default non-h-bot planning
  block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS] - position[E_AXIS]);
  block->steps_e *= volumetric_multiplier[active_extruder];
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;
  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

  Serial.println(block->steps_x);
  Serial.println(block->steps_y);
  Serial.println(block->steps_z);
  Serial.println(block->steps_e);
  Serial.println(block->step_event_count);

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments)
  {
    Serial.println("This is a zero-length block. Exiting plan_buffer_line()");
    return;
  }

  block->fan_speed = fanSpeed;

  // Compute direction bits for this block
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS])
  {
    block->direction_bits |= (1 << X_AXIS);
  }
  if (target[Y_AXIS] < position[Y_AXIS])
  {
    block->direction_bits |= (1 << Y_AXIS);
  }
  if (target[Z_AXIS] < position[Z_AXIS])
  {
    block->direction_bits |= (1 << Z_AXIS);
  }
  if (target[E_AXIS] < position[E_AXIS])
  {
    block->direction_bits |= (1 << E_AXIS);
  }
  Serial.println(block->direction_bits, BIN);

  block->active_extruder = extruder;

  if (block->steps_x != 0) Serial.println("X-axis motor enabled.");
  if (block->steps_y != 0) Serial.println("Y-axis motor enabled.");
  if (block->steps_e != 0) Serial.println("Extruder motor(s) enabled.");

  if (block->steps_e == 0)
  {
    if (feed_rate < mintravelfeedrate) feed_rate = mintravelfeedrate;
  }
  else
  {
    if (feed_rate < minimumfeedrate) feed_rate = minimumfeedrate;
  }

  float delta_mm[4];
  delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS] - position[E_AXIS]) / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[active_extruder] * extrudemultiply / 100.0;

  for (int8_t i = 0; i < 4; i++)
  {
    if (target[i])
    {
      Serial.print("Delta mm: ");
      Serial.print(axis_codes[i]);
      Serial.print(" ");
      Serial.println(delta_mm[i]);
    }
  }

  if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments )
  {
    Serial.println("Extrusion only detected");
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else
  {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }

  float inverse_millimeters = 1.0 / block->millimeters; // Inverse millimeters to remove multiple divides

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

  //  segment time im micro seconds
  unsigned long segment_time = lround(1000000.0 / inverse_second);
  if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5)))
  {
    if (segment_time < minsegmenttime)
    { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
      inverse_second = 1000000.0 / (segment_time + lround(2 * (minsegmenttime - segment_time) / moves_queued));
    }
  }

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < 4; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if (fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed
  if ( speed_factor < 1.0)
  {
    for (unsigned char i = 0; i < 4; i++)
    {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if (((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }

  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

  // Start with a safe speed
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  if (fabs(current_speed[Z_AXIS]) > max_z_jerk / 2)
    vmax_junction = min(vmax_junction, max_z_jerk / 2);
  if (fabs(current_speed[E_AXIS]) > max_e_jerk / 2)
    vmax_junction = min(vmax_junction, max_e_jerk / 2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS] - previous_speed[X_AXIS]), 2) + pow((current_speed[Y_AXIS] - previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk / jerk);
    }
    if (fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_z_jerk / fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    }
    if (fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk / fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    }
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  if (block->nominal_speed <= v_allowable) {
    block->nominal_length_flag = true;
  }
  else {
    block->nominal_length_flag = false;
  }

  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;
  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
  if (acceleration!=0) {
    return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
      (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

