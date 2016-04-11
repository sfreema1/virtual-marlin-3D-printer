#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "Arduino.h"


static bool check_endstops = true;

void microstep_init()
{
  const uint8_t microstep_modes[] = MICROSTEP_MODES;
}

void st_init()
{
  microstep_init(); //Initialize Microstepping Pins
  Serial.println("Initializing directional pins ...");
  Serial.println("Initializing stepper enable pins ...");
  Serial.println("Setting up endstops and pullup pins ...");
  Serial.println("Initializing step pins ...");

  Serial.println("Setting up waveform generation bytes");

  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;

  Serial.println("Enabling stepper driver interrupts ...");
  enable_endstops(true);
  sei();
}

void enable_endstops(bool check)
{
  check_endstops = check;
}
