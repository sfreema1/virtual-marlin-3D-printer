#include "planner.h"
#include "Configuration.h"
#include "Configuration_adv.h"
#include "ConfigurationStore.h"
#include "Marlin.h"
#include "Arduino.h"

#define EEPROM_OFFSET 100
#define EEPROM_VERSION "V11"

void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
  do
  {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  }
  while (--size);
}

#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

void Config_PrintSettings()
{ // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
  // Report steps per unit for axes
  Serial.println("Steps per unit:");
  Serial.print("M92 X");
  Serial.print(axis_steps_per_unit[0]);
  Serial.print(" Y");
  Serial.print(axis_steps_per_unit[1]);
  Serial.print(" Z");
  Serial.print(axis_steps_per_unit[2]);
  Serial.print(" E");
  Serial.println(axis_steps_per_unit[3]);

  // Report maximum feedrates (mm/s) for axes:
  Serial.println("Maximum feedrates (mm/s):");
  Serial.print("M203 X");
  Serial.print(max_feedrate[0]);
  Serial.print(" Y");
  Serial.print(max_feedrate[1]);
  Serial.print(" Z");
  Serial.print(max_feedrate[2]);
  Serial.print(" E");
  Serial.println(max_feedrate[3]);

  // Report maximum acceleration (mm/s2) for axes:
  Serial.println("Maximum acceleration (mm/s2):");
  Serial.print("M201 X");
  Serial.print(max_acceleration_units_per_sq_second[0]);
  Serial.print(" Y");
  Serial.print(max_acceleration_units_per_sq_second[1]);
  Serial.print(" Z");
  Serial.print(max_acceleration_units_per_sq_second[2]);
  Serial.print(" E");
  Serial.println(max_acceleration_units_per_sq_second[3]);

  // Report acceleration and retract acceleration for printer moves:
  Serial.println("Acceleration (S = Acceleration, T = Retract Acceleration):");
  Serial.print("M204 S");
  Serial.print(acceleration);
  Serial.print(" T");
  Serial.println(retract_acceleration);

  // Report advanced variables
  Serial.println("Advanced variables:");
  Serial.println("S = Min feedrate (mm/s), T = Min travel feedrate (mm/s), B = Min segment time (ms)");
  Serial.println("X = Max XY jerk, Z = Max Z jerk, E = Max E jerk (mm/s)");
  Serial.print("M205 S");
  Serial.print(minimumfeedrate);
  Serial.print(" T");
  Serial.print(mintravelfeedrate);
  Serial.print(" B");
  Serial.print(minsegmenttime);
  Serial.print(" X");
  Serial.print(max_xy_jerk);
  Serial.print(" Z");
  Serial.print(max_z_jerk);
  Serial.print(" E");
  Serial.println(max_e_jerk);
  
  // Report home offsets
  Serial.println("Home offsets (mm):");
  Serial.print("M206 X");
  Serial.print(add_homeing[0]);
  Serial.print(" Y");
  Serial.print(add_homeing[1]);
  Serial.print(" Z");
  Serial.println(add_homeing[2]);
  
}


/*EEPROM_SETTINGS*/
void Config_RetrieveSettings()
{
  Serial.println("Looking for previously stored configuration settings in EEPROM...");
  int i = EEPROM_OFFSET;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver);
  if (strncmp(ver, stored_ver, 3) == 0)
  {
    Serial.println("Previously stored configuration settings in the EEPROM have been detected and loaded.");
    // version number match
    EEPROM_READ_VAR(i, axis_steps_per_unit);
    EEPROM_READ_VAR(i, max_feedrate);
    EEPROM_READ_VAR(i, max_acceleration_units_per_sq_second);

    // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
    reset_acceleration_rates();

    EEPROM_READ_VAR(i, acceleration);
    EEPROM_READ_VAR(i, retract_acceleration);
    EEPROM_READ_VAR(i, minimumfeedrate);
    EEPROM_READ_VAR(i, mintravelfeedrate);
    EEPROM_READ_VAR(i, minsegmenttime);
    EEPROM_READ_VAR(i, max_xy_jerk);
    EEPROM_READ_VAR(i, max_z_jerk);
    EEPROM_READ_VAR(i, max_e_jerk);
    EEPROM_READ_VAR(i, add_homeing);
    int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
    int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
#ifndef PIDTEMP
    float Kp, Ki, Kd;
#endif
    // do not need to scale PID values as the values in EEPROM are already scaled
    EEPROM_READ_VAR(i, Kp);
    EEPROM_READ_VAR(i, Ki);
    EEPROM_READ_VAR(i, Kd);
    int lcd_contrast;
    EEPROM_READ_VAR(i, lcd_contrast);

  }
  else
  {
    Serial.println("No previous configuration settings have been detected. Default settings will be loaded.");
    Config_ResetDefault();
  }
  Config_PrintSettings();
}

void Config_ResetDefault()
{
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  long tmp3[] = DEFAULT_MAX_ACCELERATION;
  for (short i = 0; i < 4; i++)
  {
    axis_steps_per_unit[i] = tmp1[i];
    max_feedrate[i] = tmp2[i];
    max_acceleration_units_per_sq_second[i] = tmp3[i];
  }

  // steps per sq second need to be updated to agree with the units per sq second
  reset_acceleration_rates();

  acceleration = DEFAULT_ACCELERATION;
  retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime = DEFAULT_MINSEGMENTTIME;
  mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk = DEFAULT_XYJERK;
  max_z_jerk = DEFAULT_ZJERK;
  max_e_jerk = DEFAULT_EJERK;
  add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;

  Serial.println("Hardcoded default configuration settings have finished loading.");

}
