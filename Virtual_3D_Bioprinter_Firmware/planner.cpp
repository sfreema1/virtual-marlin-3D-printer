#include "Arduino.h"
#include "planner.h"
#include "Configuration.h"
#include "Language.h"
#include "Buffer.h"

/*===== Variables =====*/
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now
float axis_steps_per_unit[4];                       // Motor steps per mm
long position[4];                                   // The current position of the tool in absolute steps
float mintravelfeedrate;
float minimumfeedrate;

static int8_t next_block_index(int8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE)
  {
    block_index = 0;
  }
  return (block_index);
}

/*===== Routines =====*/
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);
  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head)
  {
    // manage_inactivity();
  }
  // The target position of the tool in absolute steps
  SERIAL_PROTOCOLLNPGM(MSG_CALCULATE_ABSOLUTE_STEPS);
  long target[4];
  target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e * axis_steps_per_unit[E_AXIS]);

  SERIAL_PROTOCOLPAIR_COLON(axis_codes[X_AXIS], target[X_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON(axis_codes[Y_AXIS], target[Y_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON(axis_codes[Z_AXIS], target[Z_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON(axis_codes[E_AXIS], target[E_AXIS]);
  SERIAL_PROTOCOL_END;

  //Ignoring portion on preventing dangerous and lengthy extrusions

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  SERIAL_PROTOCOLLNPGM(MSG_CREATE_NEW_BLOCK);
  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  //Calculate relative steps to target
  SERIAL_PROTOCOLLNPGM(MSG_CALCULATE_RELATIVE_STEPS);
  block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS] - position[E_AXIS]);
  block->steps_e *= volumetric_multiplier[active_extruder];
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;

  SERIAL_PROTOCOLPAIR_COLON("X", block->steps_x);
  SERIAL_PROTOCOLPAIR_COLON("Y", block->steps_y);
  SERIAL_PROTOCOLPAIR_COLON("Z", block->steps_z);
  SERIAL_PROTOCOLPAIR_COLON("E", block->steps_e);
  SERIAL_PROTOCOL_END;

  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));
  SERIAL_PROTOCOLPAIR_COLON(MSG_MAX_STEP_EVENT, block->step_event_count);

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments)
  {
    SERIAL_PROTOCOLLNPGM(MSG_STEP_COUNT_ERROR);
    return;
  }

  // Compute direction bits for this block
  SERIAL_PROTOCOLLNPGM(MSG_CALCULATE_DIRECTIONS);
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
  if (block->steps_x != 0) SERIAL_PROTOCOLLNPGM(MSG_ENABLE_X_AXIS);
  if (block->steps_y != 0) SERIAL_PROTOCOLLNPGM(MSG_ENABLE_Y_AXIS);
  if (block->steps_z != 0) SERIAL_PROTOCOLLNPGM(MSG_ENABLE_Z_AXIS);
  if (block->steps_e != 0) SERIAL_PROTOCOLLNPGM(MSG_ENABLE_E_AXIS);

  if (block->steps_e == 0)
  {
    if (feed_rate < mintravelfeedrate)
    {
      feed_rate = mintravelfeedrate;
      SERIAL_PROTOCOLLNPGM(MSG_FEEDRATE_SET_TO_MIN_TRAVELFEEDRATE);
    }
  }
  else
  {
    if (feed_rate < minimumfeedrate)
    {
      feed_rate = minimumfeedrate;
      SERIAL_PROTOCOLLNPGM(MSG_FEEDRATE_SET_TO_MIN_FEEDRATE);
    }
  }


  float delta_mm[4];
  SERIAL_PROTOCOLLNPGM(MSG_CALCULATE_RELATIVE_DISTANCE);
  delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS] - position[E_AXIS]) / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[active_extruder] * extrudemultiply / 100.0;

  SERIAL_PROTOCOLPAIR_COLON("X", delta_mm[X_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON("Y", delta_mm[Y_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON("Z", delta_mm[Z_AXIS]);
  SERIAL_PROTOCOLPAIR_COLON("E", delta_mm[E_AXIS]);
  SERIAL_PROTOCOL_END;

  SERIAL_PROTOCOLLNPGM(MSG_CALCULATE_TOTAL_EUCLIDEAN_DISTANCE);
  if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments )
  {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else
  {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  SERIAL_PROTOCOLPAIR_COLON("Distance: ", block->millimeters);
  SERIAL_PROTOCOL_END;

  float inverse_millimeters = 1.0 / block->millimeters; // Inverse millimeters to remove multiple divides
  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters; // Inverse seconds to remove multiple divides

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
  // ADD TO SHOW MOVES QUEUED

  // Ignoring slowdown section

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0 // Without slowdown modifications it is the feedrate
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0 // max steps over same amount of time as nominal speed

  // Calculate and limit speed in mm/sec for each axis
  // This calculation seems arbitrary since speed factor is always choosen as the least among either the current speed factor or new one
  float current_speed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < 4; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second; // speed for each axis
    if (fabs(current_speed[i]) > max_feedrate[i]) speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed
  if ( speed_factor < 1.0)
  {
    for (unsigned char i = 0; i < 4; i++)
    {
      current_speed[i] *= speed_factor;
    }
    // Adjust speed and rate by speed_factor
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  /*else
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
  }*/

}

