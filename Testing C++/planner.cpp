//
//  planner.cpp
//  Testing C++
//
//  Created by Sebastian Freeman on 4/9/16.
//  Copyright © 2016 Sebastian Freeman. All rights reserved.
//

#include "planner.hpp"
#include "Configuration.h"
#include <iostream>
#include <math.h>
#include <algorithm>    // std::max & std::min
#include <bitset>       // std::bitset

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
float axis_steps_per_unit[4] = {80.0,80.0,400.0,500.0};
float max_feedrate[4] = {500, 500, 5, 25};
float mintravelfeedrate = 0;
float minimumfeedrate = 0;
float retract_acceleration = 3000;
float acceleration = 3000;
float axis_steps_per_sqr_second[4] = {9000,9000,100,500};
long position[4] = {0,0,0,0};
float max_xy_jerk = 20.0;
float max_z_jerk = 0.4;
float max_e_jerk = 5.0;

static float previous_speed[4] = {0.,0.,0.,0.};
static float previous_nominal_speed = 0;

block_t block_buffer[BLOCK_BUFFER_SIZE];
volatile unsigned char block_buffer_head = 0;
volatile unsigned char block_buffer_tail = 0;

// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
    if (block_index == 0) {
        block_index = BLOCK_BUFFER_SIZE;
    }
    block_index--;
    return(block_index);
}

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
    block_index++;
    if (block_index == BLOCK_BUFFER_SIZE) {
        block_index = 0;
    }
    return(block_index);
}

FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
    return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration){
    if (acceleration!=0) {
        return((target_rate*target_rate-initial_rate*initial_rate)/(2.0*acceleration));
    }
    else {
        return 0.0;  // acceleration was 0, set acceleration distance to 0
    }
}

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance){
    if (acceleration!=0) {
        return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
               (4.0*acceleration) );
    }
    else {
        return 0.0;  // acceleration was 0, set intersection distance to 0
    }
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
    if(!current) {
        PRINTLN("Exiting the planner_reverse_pass_kernel since there is no current block...");
        return;
    }
    
    if (next) {
        PRINTLN("There is a next block in the buffer ...");
        // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
        // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
        // check for maximum allowable speed reductions to ensure maximum possible planned speed.
        if (current->entry_speed != current->max_entry_speed) {
            
            // If nominal length true, max junction speed is guaranteed to be reached. Only compute
            // for max allowable speed if block is decelerating and nominal length is false.
            if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
                current->entry_speed = std::min( current->max_entry_speed, max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
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
void planner_reverse_pass() {
    PRINTLN("");
    PRINTLN("Beginning the planner_reverse_pass() routine ...");
    uint8_t block_index = block_buffer_head;
    
    //Make a local copy of block_buffer_tail, because the interrupt can alter it
    CRITICAL_SECTION_START;
    unsigned char tail = block_buffer_tail;
    CRITICAL_SECTION_END;
    std::cout << "The current and tail indices being used in this routine are: " << block_index << " and " << tail << "\n";
    
    if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
        PRINTLN("There are more than three blocks in the block buffer...");
        block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
        block_t *block[3] = {NULL, NULL, NULL};
        while(block_index != tail) {
            PRINTLN("The planner_reverse_pass_kernel() routine will be run ...");
            block_index = prev_block_index(block_index);
            block[2]= block[1];
            block[1]= block[0];
            block[0] = &block_buffer[block_index];
            planner_reverse_pass_kernel(block[0], block[1], block[2]);
        }
    }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
    if(!previous) {
        PRINTLN("Exiting the planner_forward_pass_kernel since there is no previous block...");
        return;
    }
    
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag) {
        if (previous->entry_speed < current->entry_speed) {
            double entry_speed = std::min(current->entry_speed,max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters));
            
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
    PRINTLN("");
    PRINTLN("Beginning the planner_forward_pass() routine ...");
    uint8_t block_index = block_buffer_tail;
    block_t *block[3] = {NULL, NULL, NULL};
    MSGLN("The current block index is ",block_index);
    
    while(block_index != block_buffer_head) {
        PRINTLN("Moving up the block buffer...");
        block[0] = block[1];
        block[1] = block[2];
        block[2] = &block_buffer[block_index];
        planner_forward_pass_kernel(block[0],block[1],block[2]);
        block_index = next_block_index(block_index);
    }
    planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids() {
    PRINTLN("");
    PRINTLN("Beginning the planner_recalculate_trapezoids() routine ...");
    int8_t block_index = block_buffer_tail;
    block_t *current;
    block_t *next = NULL;
    
    while(block_index != block_buffer_head) {
        PRINTLN("Moving up the block buffer ...");
        current = next;
        next = &block_buffer[block_index];
        if (current) {
            // Recalculate if current block entry or exit junction speed has changed.
            if (current->recalculate_flag || next->recalculate_flag) {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed, next->entry_speed/current->nominal_speed);
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            }
        }
        block_index = next_block_index( block_index );
    }
    // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
    if(next != NULL) {
        calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,MINIMUM_PLANNER_SPEED/next->nominal_speed);
        next->recalculate_flag = false;
    }
}

void planner_recalculate() {
    PRINTLN("");
    PRINTLN("Beginning the planner_recalculate() routine");
    PRINTLN("In this routine we will pass over the current moves 1) reverse, 2) forward, 3) and finally recalculate all trapezoids.");
    planner_reverse_pass();
    planner_forward_pass();
    planner_recalculate_trapezoids();
}


void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
    PRINT("\n");
    PRINTLN("Begining the calculate_trapezoid_for_block() routine ...");
    unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
    MSGLN("The initial rate for this block will be (step/min): ", initial_rate);
    unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)
    MSGLN("The final rate for this block will be (step/min): ", final_rate);
    
    // Limit minimal step rate (Otherwise the timer will overflow.)
    if(initial_rate <120) {
        initial_rate=120;
        PRINTLN("The initial rate has been limited to 120 step/min.");
    }
    if(final_rate < 120) {
        final_rate=120;
        PRINTLN("The final rate has been limited to 120 step/min.");
    }
    
    long acceleration = block->acceleration_st;
    int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
    MSGLN("The number of steps for the acceleration will be ", accelerate_steps);
    int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));
    MSGLN("The number of steps for the deceleration will be ", decelerate_steps);
    
    // Calculate the size of Plateau of Nominal Rate.
    int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
    MSGLN("The number of plateau steps for the block are ", plateau_steps);
    
    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        PRINTLN("Since plateau steps are less than 0, we need to calculate the intersection distance.");
        accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
        accelerate_steps = std::max(accelerate_steps,0); // Check limits due to numerical round-off
        PRINTLN(block->step_event_count);
        accelerate_steps = std::min((uint32_t)accelerate_steps,(uint32_t)block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
        plateau_steps = 0;
        PRINTLN(block->step_event_count);
    }
    CRITICAL_SECTION_START;
    if(block->busy == false) { // Don't update variables if block is busy.
        block->accelerate_until = accelerate_steps;
        MSGLN("The block will accelerate for (steps) ", block->accelerate_until);
        block->decelerate_after = accelerate_steps+plateau_steps;
        MSGLN("The block begin to decelerate after (steps) ", block->decelerate_after);
        block->initial_rate = initial_rate;
        MSGLN("The initial rate is: ", block->initial_rate);
        block->final_rate = final_rate;
        MSGLN("The final rate is: ", block->final_rate);
    }
    CRITICAL_SECTION_END;
}

void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder){
    
    // Calculate the buffer head after we push this byte
    int next_buffer_head = next_block_index(block_buffer_head);
    
    
    // If the buffer is full: good! That means we are well ahead of the robot.
    // Rest here until there is room in the buffer.
    while(block_buffer_tail == next_buffer_head)
    {
        PRINTLN("Managing inactivity !!!!!!!!!!!!!!!!!!!!!!!... ");
        //manage_heater();
        check_axes_activity();
        //lcd_update();
    }
    
    long target[4];
    target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
    target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
    target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);
    target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
    PRINT("\n");
    PRINTLN("Beginning the plan_buffer_line() routine ...");
    
    MSGLN("The target absolute step x-location is ",target[0]);
    MSGLN("The target absolute step y-location is ",target[1]);
    MSGLN("The target absolute step z-location is ",target[2]);
    MSGLN("The target absolute step e-location is ",target[3]);
    
    block_t *block = &block_buffer[block_buffer_head];
    if (block_buffer_head == 0){
        PRINTLN("The current block will be made on the block buffer head index: 0");
    }
    else{
        MSGLN("The current block will be made on the block buffer head index: ",block_buffer_head);
    }
    
    // Mark block as not busy (Not executed by the stepper interrupt)
    block->busy = false;
    
    block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
    block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
    block->steps_y = labs(target[Z_AXIS]-position[Z_AXIS]);
    block->steps_z = labs(target[E_AXIS]-position[E_AXIS]);
    
    MSGLN("The steps needed to reach the target x-location from the current location is ",block->steps_x);
    MSGLN("The steps needed to reach the target y-location from the current location is ",block->steps_y);
    MSGLN("The steps needed to reach the target z-location from the current location is ",block->steps_z);
    MSGLN("The steps needed to reach the target e-location from the current location is ",block->steps_e);
    
    block->step_event_count = std::max(block->steps_x, std::max(block->steps_y, std::max(block->steps_z, block->steps_e)));
    MSGLN("The most steps needed for any move are ", block->step_event_count);
    
    // Bail if this is a zero-length block
    if (block->step_event_count <= dropsegments){
        PRINTLN("The step event count is less than the dropsegment parameter. Aborting planning...");
        return;
    }

    // Compute direction bits for this block
    block->direction_bits = 0;
    if (target[X_AXIS] < position[X_AXIS]){
        block->direction_bits |= (1<<X_AXIS);
        PRINTLN("A negative x-move!");
    }
    if (target[Y_AXIS] < position[Y_AXIS]){
        block->direction_bits |= (1<<Y_AXIS);
        PRINTLN("A negative y-move!");
    }
    if (target[Z_AXIS] < position[Z_AXIS]){
        block->direction_bits |= (1<<Z_AXIS);
        PRINTLN("A negative z-move!");
    }
    if (target[E_AXIS] < position[E_AXIS]){
        block->direction_bits |= (1<<E_AXIS);
        PRINTLN("A negative e-move!");
    }
    
    std::bitset<8> db(block->direction_bits);
    MSGLN("The calculated direction bit is ", db);
    
    block->active_extruder = extruder;
    
    if (block->steps_e == 0)
    {
        if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
    }
    else
    {
        if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
    }
    
    float delta_mm[4];
    delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
    delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
    delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS]);
    MSGLN("The gantry will move in the x-direction (mm): ", delta_mm[X_AXIS]);

    if ( block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments ){
        block->millimeters = fabs(delta_mm[E_AXIS]);
        MSGLN("Only moving the extruder. It will move (mm):  ", block->millimeters);
    }
    else{
        block->millimeters = sqrt(pow(delta_mm[X_AXIS],2) + pow(delta_mm[Y_AXIS],2) + pow(delta_mm[Z_AXIS],2));
        MSGLN("The total distance to be moved is (mm): ", block->millimeters);
    }
    
    float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides
    
    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
    float inverse_second = feed_rate * inverse_millimeters;
    MSGLN("Based on the given feedrate and the distance, the move will take (s): ", 1.0/inverse_second);
    
    int moves_queued = (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
    MSGLN("The number of queued moves are ", moves_queued);
    
    block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
    block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0
    MSGLN("The nominal speed is (mm/s): ", block->nominal_speed);
    MSGLN("The nominal rate is (steps/s): ", block->nominal_rate);
    
    // Calculate and limit speed in mm/sec for each axis
    float current_speed[4];
    float speed_factor = 1.0; //factor <=1 do decrease speed
    for(int i=0; i < 4; i++){
        current_speed[i] = delta_mm[i] * inverse_second;
        std::cout<<"The proposed speed for the "<<axis_codes[i]<<" axis is "<<current_speed[i]<<std::endl;
        if(fabs(current_speed[i]) > max_feedrate[i]){
            speed_factor = std::min(speed_factor, max_feedrate[i] / fabsf(current_speed[i]));
        }
        MSGLN("The minimum speed factor to use for the axes so far is ", speed_factor);
    }
    
    // Correct the speed
    if( speed_factor < 1.0)
    {
        for(unsigned char i=0; i < 4; i++)
        {
            current_speed[i] *= speed_factor;
            std::cout<<"The current speed (mm/s) for the "<<axis_codes[i]<<" axis is "<<current_speed[i]<<std::endl;
        }
        block->nominal_speed *= speed_factor;
        block->nominal_rate *= speed_factor;
        MSGLN("The final nominal speed is (mm/s): ", block->nominal_speed);
        MSGLN("The final nominal rate is (steps/s): ", block->nominal_rate);
    }
    
    // Compute and limit the acceleration rate for the trapezoid generator.
    float steps_per_mm = block->step_event_count/block->millimeters;
    MSGLN("The nominal steps per mm is: ", steps_per_mm);
    if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0){
        
        block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
        MSGLN("The only steps necessary for the move are for the extruder. The step acceleration rate is (steps/s^2): ", block->acceleration_st);
    }
    else{
        block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
        MSGLN("The proposed acceleration rate (steps/s^2) is ", block->acceleration_st);
        // Limit acceleration per axis
        if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS]){
            block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
            MSGLN("The x-motion has limited the step acceleration rate. New value is (steps/s^2):" , block->acceleration_st);
        }
        if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS]){
            block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
            MSGLN("The y-motion has limited the step acceleration rate. New value is (steps/s^2):" , block->acceleration_st);
        }
        if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS]){
            block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
            MSGLN("The e-motion has limited the step acceleration rate. New value is (steps/s^2):" , block->acceleration_st);
        }
        if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS]){
            block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
            MSGLN("The z-motion has limited the step acceleration rate. New value is (steps/s^2):" , block->acceleration_st);
        }
    }
    
    block->acceleration = block->acceleration_st / steps_per_mm;
    MSGLN("The acceleration for the move is (mm/s^2): ", block->acceleration);
    block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));
    MSGLN("The acceleration rate is ", block->acceleration_rate);
    
    float vmax_junction = max_xy_jerk/2;
    float vmax_junction_factor = 1.0;
    
    if(fabsf(current_speed[Z_AXIS]) > max_z_jerk/2){
        vmax_junction = std::min(vmax_junction, max_z_jerk/2);
        MSGLN("The current z-speed exceeds half the maximum z jerk. The maximum junction velocity has been changed to: ", vmax_junction);
    }
    if(fabs(current_speed[E_AXIS]) > max_e_jerk/2){
        vmax_junction = std::min(vmax_junction, max_e_jerk/2);
        MSGLN("The current e-speed exceeds half the maximum e jerk. The maximum junction velocity has been changed to: ", vmax_junction);
    }
    vmax_junction = std::min(vmax_junction, block->nominal_speed);
    MSGLN("In the end, the maximum junction velocity is (mm/s): ", vmax_junction);
    float safe_speed = vmax_junction;
    PRINTLN("A safe speed flag has been set with the current maximum junction velocity.");
    if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
        float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
        MSGLN("Since there have been previous moves, set a jerk speed of :", jerk);

        vmax_junction = block->nominal_speed;
        MSGLN("The maximum junction velocity has been set ot the nominal speed: ", vmax_junction);
        if (jerk > max_xy_jerk) {
            vmax_junction_factor = (max_xy_jerk/jerk);
        }
        if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
            vmax_junction_factor= std::min(vmax_junction_factor, (max_z_jerk/fabsf(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
        }
        if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
            vmax_junction_factor = std::min(vmax_junction_factor, (max_e_jerk/fabsf(current_speed[E_AXIS] - previous_speed[E_AXIS])));
        }
        vmax_junction = std::min(previous_nominal_speed, vmax_junction * vmax_junction_factor);
        MSGLN("The the maximum velocity junction factor has been set to ",vmax_junction_factor);
    }
    block->max_entry_speed = vmax_junction;
    MSGLN("The max entry speed has been set to (mm/s): ", block->max_entry_speed);
    
    double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
    MSGLN("The max allowable speed is (mm/s): ", v_allowable);
    
    block->entry_speed = std::min((double)vmax_junction, v_allowable);
    MSGLN("The chosen entry speed will be (mm/s): ", block->entry_speed);
    
    if (block->nominal_speed <= v_allowable) {
        block->nominal_length_flag = true;
        PRINTLN("The nominal length flag has been set since the nominal speed is less than or equal to the allowable speed.");
    }
    else {
        block->nominal_length_flag = false;
    }
    block->recalculate_flag = true; // Always calculate trapezoid for new block
    
    // Update previous path unit_vector and nominal speed
    memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
    previous_nominal_speed = block->nominal_speed;
    calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, safe_speed/block->nominal_speed);
    
    // Move buffer head
    MSGLN("The next buffer head index will be ",next_buffer_head);
    block_buffer_head = next_buffer_head;
    // Update position
    memcpy(position, target, sizeof(target)); // position[] = target[]
    planner_recalculate();
}

void check_axes_activity()
{
    PRINTLN("");
    PRINTLN("Beginning check_axes_activity() routine...");
    unsigned char x_active = 0;
    unsigned char y_active = 0;
    unsigned char z_active = 0;
    unsigned char e_active = 0;
    // unsigned char tail_fan_speed = fanSpeed;
    block_t *block;
    
    if(block_buffer_tail != block_buffer_head)
    {
        PRINTLN("The block tail is not equal to the head ...");
        uint8_t block_index = block_buffer_tail;
        //tail_fan_speed = block_buffer[block_index].fan_speed;
        while(block_index != block_buffer_head)
        {
            block = &block_buffer[block_index];
            if(block->steps_x != 0) x_active++;
            if(block->steps_y != 0) y_active++;
            if(block->steps_z != 0) z_active++;
            if(block->steps_e != 0) e_active++;
            block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
        }
    }
    if((DISABLE_X) && (x_active == 0)) disable_x();
    if((DISABLE_Y) && (y_active == 0)) disable_y();
    if((DISABLE_Z) && (z_active == 0)) disable_z();
    if((DISABLE_E) && (e_active == 0))
    {
        disable_e0();
        disable_e1();
        disable_e2(); 
    }
}

