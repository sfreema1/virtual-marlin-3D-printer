#include "Marlin.h"
#include "planner.h"
#include "planner.h"
#include "Configuration.h"
#include "Configuration_adv.h"
#include "ConfigurationStore.h"
#include "Marlin.h"
#include "Arduino.h"

/*===== Public Variables =====*/
unsigned long minsegmenttime;
float max_feedrate[4]; // set the max speeds
float axis_steps_per_unit[4];
unsigned long max_acceleration_units_per_sq_second[4]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves.
float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis
float max_xy_jerk; //speed than can be stopped at once, if I understand correctly.
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

// The current position of the tool in absolute steps
long position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment

/*===== Semi-private variables, used in inline functions =====*/
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

/*===== Private Variables =====*/
// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) {
    block_index = 0;
  }
  return (block_index);
}

// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) {
    block_index = BLOCK_BUFFER_SIZE;
  }
  block_index--;
  return (block_index);
}

/*===== Routines =====*/

FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance)
{
  return  sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration != 0) {
    return ((target_rate * target_rate - initial_rate * initial_rate) /
            (2.0 * acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
{
  if (acceleration != 0) {
    return ((2.0 * acceleration * distance - initial_rate * initial_rate + final_rate * final_rate) /
            (4.0 * acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate * entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate * exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if (initial_rate < 120) {
    initial_rate = 120;
  }
  if (final_rate < 120) {
    final_rate = 120;
  }

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count); //(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }
  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates()
{
  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!current) {
    return;
  }

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
                                    max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
      }
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass()
{
  uint8_t block_index = block_buffer_head;

  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if (((block_buffer_head - tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = {
      NULL, NULL, NULL
    };
    while (block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed / current->nominal_speed,
                                      next->entry_speed / current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed / next->nominal_speed,
                                  MINIMUM_PLANNER_SPEED / next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!previous) {
    return;
  }

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
                                max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = {
    NULL, NULL, NULL
  };

  while (block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0], block[1], block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

void planner_recalculate()
{
  planner_reverse_pass();
  planner_forward_pass();
  // planner_recalculate_trapezoids();
}

void plan_init()
{
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
  Serial.println("Initializing planner ..."); Serial.println("Block buffer, XYZE position, and speeds have been set cleared.");
}

void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);
  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head)
  {
    Serial.println("Block buffer is full and cannot accept another block. In the meantime Marlin will continue all other operations.");
  }
  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[4];
  target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);
  Serial.println("Calculating motor steps from origin each axis to reach destination ... ");
  for (int8_t i = 0; i < 4; i++)
  {

    if (target[i])
    {
      Serial.print("Number of steps for "); Serial.print(axis_codes[i]); Serial.print(": "); Serial.println(target[i]);
    }
  }

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;
  Serial.println("Setting up new block struct ...");
  Serial.println("Calculating motor steps from current position to destination ...");
  block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
  if (block->steps_x)
  {
    Serial.print("△"); Serial.print(axis_codes[0]); Serial.print(": "); Serial.println(block->steps_x);
  }
  block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
  if (block->steps_y)
  {
    Serial.print("△"); Serial.print(axis_codes[1]); Serial.print(": "); Serial.println(block->steps_y);
  }
  block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
  if (block->steps_z)
  {
    Serial.print("△"); Serial.print(axis_codes[2]); Serial.print(": "); Serial.println(block->steps_z);
  }
  block->steps_e = labs(target[E_AXIS] - position[E_AXIS]);
  if (block->steps_e)
  {
    Serial.print("△"); Serial.print(axis_codes[3]); Serial.print(": "); Serial.println(block->steps_e);
  }
  block->steps_e *= volumetric_multiplier[active_extruder];
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;
  Serial.println(block->steps_e);
  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));
  Serial.print("Greatest number of steps among all axes: ");
  Serial.println(block->step_event_count);

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments)
  {
    Serial.println("Max steps for planned motion is less than minimum allowed. Aborting buffer line planning ...");
    return;
  }

  block->fan_speed = fanSpeed;
  Serial.println("Calculating necessary motor directions for requested destination ...");
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
  block->active_extruder = extruder;

  //enable active axes
  if (block->steps_x != 0) Serial.println("Enabling X axis motor ...");
  if (block->steps_y != 0) Serial.println("Enabling Y axis motor ...");
  if (block->steps_z != 0) Serial.println("Enabling Z axis motor ...");
  if (block->steps_e != 0) Serial.println("Enabling extruder motor ...");

  if (block->steps_e == 0)
  {
    Serial.println("No extrusion requested. If feedrate is less than min travel feedrate, it will be set to equal it. ");
    if (feed_rate < mintravelfeedrate)
    {
      feed_rate = mintravelfeedrate;
      Serial.print("Feedrate set to min travel feedrate: "); Serial.println(feed_rate);
    }
  }
  else
  {
    Serial.println("Extrusion requested. If feedrate is less than min feedrate, it will be set to equal it. ");
    if (feed_rate < minimumfeedrate)
    {
      feed_rate = minimumfeedrate;
      Serial.print("Feedrate set to min feedrate: "); Serial.println(feed_rate);
    }
  }
  Serial.print("Feedrate: "); Serial.println(feed_rate);

  Serial.println("Calculating distance in mm from current position to reach destination ...");
  float delta_mm[4];
  delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / axis_steps_per_unit[X_AXIS];
  if (delta_mm[X_AXIS])
  {
    Serial.print("del_"); Serial.print(axis_codes[0]); Serial.print(": "); Serial.print(delta_mm[X_AXIS]); Serial.println(" mm");
  }
  delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / axis_steps_per_unit[Y_AXIS];
  if (delta_mm[Y_AXIS])
  {
    Serial.print("del_"); Serial.print(axis_codes[1]); Serial.print(": "); Serial.print(delta_mm[Y_AXIS]); Serial.println(" mm");
  }
  delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / axis_steps_per_unit[Z_AXIS];
  if (delta_mm[Z_AXIS])
  {
    Serial.print("del_"); Serial.print(axis_codes[2]); Serial.print(": "); Serial.print(delta_mm[Z_AXIS]); Serial.println(" mm");
  }
  delta_mm[E_AXIS] = ((target[E_AXIS] - position[E_AXIS]) / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[active_extruder] * extrudemultiply / 100.0;
  if (delta_mm[E_AXIS])
  {
    Serial.print("del_"); Serial.print(axis_codes[3]); Serial.print(": "); Serial.print(delta_mm[E_AXIS]); Serial.println(" mm");
  }

  if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments )
  {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else
  {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  Serial.print("Total Euclidean distance: "); Serial.print(block->millimeters); Serial.println(" mm");

  float inverse_millimeters = 1.0 / block->millimeters; // Inverse millimeters to remove multiple divides

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
  Serial.print("Number of moves queued: "); Serial.println(moves_queued);

  //  segment time im micro seconds
  unsigned long segment_time = lround(1000000.0 / inverse_second);
  Serial.print("Segment will take "); Serial.print(segment_time / 1000.0); Serial.println(" ms to complete.");
  if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5)))
  {
    Serial.println("Block buffer is at less than half capacity. Adjusting moves to slower speed...");
    if (segment_time < minsegmenttime)
    { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
      // As segment_time increases, inverse_seconds decreases. As moves_queued increase, inverse_seconds increases.
      inverse_second = 1000000.0 / (segment_time + lround(2 * (minsegmenttime - segment_time) / moves_queued));
    }
  }
  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0
  Serial.println("Calculcating nominal speed and step rate ...");
  Serial.print("Nominal speed: "); Serial.print(block->nominal_speed); Serial.println(" mm/s");
  Serial.print("Nominal rate: "); Serial.print(block->nominal_rate); Serial.println(" steps/s");

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  Serial.println("Calculating speed for each axis ...");
  Serial.println("If calculated speed is greater than max feedrate, speed factor will be adjusted.");
  for (int i = 0; i < 4; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if (fabs(current_speed[i]) > max_feedrate[i]) speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
    if (current_speed[i])
    {
      Serial.print(axis_codes[i]); Serial.print("->"); Serial.print("Speed: "); Serial.print(current_speed[i]); Serial.print(" mm/s"); Serial.print(" | Speed factor: "); Serial.println(speed_factor);
    }
  }

  // Correct the speed
  if ( speed_factor < 1.0)
  {
    Serial.println("Calculated speed factor is less than one. Adjusting calculated speeds ...");
    for (unsigned char i = 0; i < 4; i++)
    {
      current_speed[i] *= speed_factor;
      if (current_speed[i])
      {
        Serial.print(axis_codes[i]); Serial.print("->"); Serial.print("Speed: "); Serial.print(current_speed[i]); Serial.println(" mm/s");
      }
    }
    Serial.println("Adjusting nominal speed and nominal rate by speed factor ...");
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
    Serial.print("Nominal speed: "); Serial.print(block->nominal_speed); Serial.println(" mm/s");
    Serial.print("Nominal rate: "); Serial.print(block->nominal_rate); Serial.println(" steps/s");
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  Serial.println("Calculating steps/mm and acceleration step rate (steps/sec^2) ...");
  Serial.println("Steps/mm is calcuated using max step count/Euclidean distance in mm.");
  float steps_per_mm = block->step_event_count / block->millimeters;
  Serial.print("Steps/mm: "); Serial.print(steps_per_mm); Serial.println(" steps/mm");
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    Serial.println("Since no displacement in XYZ is required, retract acceleration is used.");
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    Serial.print("Acceleration step rate: E->"); Serial.print(block->acceleration_st); Serial.println(" steps/sec^2");
  }
  else
  {
    Serial.println("Since displacement in XYZ is required, default acceleration will be used.");
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    Serial.print("Acceleration step rate: "); Serial.print(block->acceleration_st); Serial.println(" steps/sec^2");
    Serial.println("If acceleration step rate exceeds max, acceleration step rate values will be adjusted.");
    // Limit acceleration per axis
    if (((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
    {
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    }
    if (((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
    {
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    }
    if (((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
    {
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    }
    if (((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
    {
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
    }
    Serial.print("Acceleration step rate: "); Serial.print(block->acceleration_st); Serial.println(" steps/sec^2");
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  Serial.print("Acceleration: "); Serial.print(block->acceleration); Serial.println(" mm/sec^2");
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0))); // Takes into account the Hz of the Arduino clock
  Serial.print("Acceleration rate: "); Serial.print(block->acceleration_rate); Serial.println(" sec^-2");

  // Start with a safe speed
  Serial.println("Calculating safe speed ...");
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  if (fabs(current_speed[Z_AXIS]) > max_z_jerk / 2) vmax_junction = min(vmax_junction, max_z_jerk / 2);
  if (fabs(current_speed[E_AXIS]) > max_e_jerk / 2) vmax_junction = min(vmax_junction, max_e_jerk / 2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;
  Serial.print("Safe speed: "); Serial.print(safe_speed); Serial.println(" mm/s");

  Serial.println("Calculating max entry speed ...");
  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS] - previous_speed[X_AXIS]), 2) + pow((current_speed[Y_AXIS] - previous_speed[Y_AXIS]), 2));

    vmax_junction = block->nominal_speed;

    if (jerk > max_xy_jerk)
    {
      vmax_junction_factor = (max_xy_jerk / jerk);
    }
    if (fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk)
    {
      vmax_junction_factor = min(vmax_junction_factor, (max_z_jerk / fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    }
    if (fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk)
    {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk / fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    }
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->max_entry_speed = vmax_junction;
  Serial.print("Max entry speed: "); Serial.print(block->max_entry_speed); Serial.println(" mm/s");

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable)
  {
    Serial.println("Ideal segment length for acceleration/decceleration.");
    block->nominal_length_flag = true;
  }
  else
  {
    Serial.println("Non-ideal segment length for acceleration/decceleration.");
    block->nominal_length_flag = false;
  }

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;
  Serial.println("Calculating trapezoid for segment ...");
  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;
  Serial.println("Switching to next buffer head ...");

  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();

  Serial.println("Awaking stepper drivers ...");

}
