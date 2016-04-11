//
//  main.cpp
//  Testing C++
//
//  Created by Sebastian Freeman on 9/12/15.
//  Copyright (c) 2015 Sebastian Freeman. All rights reserved.
//

#include <iostream>
#include "planner.hpp"
#include "main.h"
#include "Configuration.h"

int main() {
    float x[2] = {200.0,250.0};
    float y[2] = {200.2,445.4};
    float z[2] = {200.3,34.5};
    float e[2] = {20.0,21.};
    float feed_rate[2] = {60., 30.};
    uint8_t active_extruder = 0;
    
    for (int i=0; i<2; i++) {
        plan_buffer_line(x[i], y[i], z[i], e[i], feed_rate[i], active_extruder);
    }
}


