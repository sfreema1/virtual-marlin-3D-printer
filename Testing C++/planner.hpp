//
//  planner.hpp
//  Testing C++
//
//  Created by Sebastian Freeman on 4/9/16.
//  Copyright © 2016 Sebastian Freeman. All rights reserved.
//

#ifndef planner_hpp
#define planner_hpp

#include <stdio.h>
#include "main.h"
#include <math.h>
#include "Configuration.h"
#include "stdint.h"

typedef struct {
    float millimeters;
    float nominal_speed;
    float acceleration;
    
    unsigned char recalculate_flag;
    unsigned char nominal_length_flag;
    volatile char busy;
    long accelerate_until;
    long decelerate_after;
    unsigned long initial_rate;
    unsigned long final_rate;
    
    long steps_x,steps_y,steps_z,steps_e;
    unsigned long nominal_rate;
    unsigned long step_event_count;
    unsigned char direction_bits;
    unsigned char active_extruder;
    long acceleration_rate;
    float max_entry_speed;
    float entry_speed; 
    
    unsigned long acceleration_st;
    
} block_t;

extern float axis_steps_per_unit[4];
extern float max_feedrate[4];
extern long position[4];
extern block_t block_buffer[BLOCK_BUFFER_SIZE];
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor);

void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder);

void check_axes_activity();

#endif /* planner_hpp */
