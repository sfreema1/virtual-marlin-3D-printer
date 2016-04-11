#ifndef STEPPER_H
#define STEPPER_H

#include "planner.h"

// Initialize and start the stepper motor subsystem
void st_init();

// Enable/disable endstop checking
void enable_endstops(bool check);

void microstep_mode(uint8_t driver, uint8_t stepping);

#endif // STEPPER_H
