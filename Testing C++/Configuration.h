//
//  Configuration.h
//  Testing C++
//
//  Created by Sebastian Freeman on 4/9/16.
//  Copyright © 2016 Sebastian Freeman. All rights reserved.
//

#ifndef Configuration_h
#define Configuration_h

#define PRINT(x) std::cout << x
#define PRINTLN(x) std::cout << x << std::endl
#define MSG(str,x) std::cout << str << x
#define MSGLN(str,x) std::cout << str << x << std::endl

#define BLOCK_BUFFER_SIZE 16

#define NUM_AXIS 4

#define F_CPU 16000000

#define  FORCE_INLINE __attribute__((always_inline)) inline

const unsigned int dropsegments = 5;

#define MINIMUM_PLANNER_SPEED 0.05

#define CRITICAL_SECTION_START PRINTLN("ENTERING CRITICAL SECTION ...");

#define CRITICAL_SECTION_END PRINTLN("EXITING CRITICAL SECTION ...");

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled

#define disable_x() PRINTLN("Disabling x motor...")
#define disable_y() PRINTLN("Disabling y motor...")
#define disable_z() PRINTLN("Disabling z motor...")
#define disable_e0() PRINTLN("Disabling e0 motor...")
#define disable_e1() PRINTLN("Disabling e1 motor...")
#define disable_e2() PRINTLN("Disabling e2 motor...")

#endif /* Configuration_h */
