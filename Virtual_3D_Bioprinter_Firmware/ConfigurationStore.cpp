#include "Arduino.h"
#include "Buffer.h"
#include "Planner.h"
#include "Language.h"
#include "ConfigurationStore.h"


void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
  do
  {
    eeprom_write_byte((unsigned char*)pos, *value);
    pos++;
    value++;
  } while (--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))

void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
  do
  {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  } while (--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

#define EEPROM_OFFSET 100
#define EEPROM_VERSION "V11"

void Config_StoreSettings()
{
  char ver[4] = "000";
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver); // invalidate data first
  EEPROM_WRITE_VAR(i, axis_steps_per_unit);
  EEPROM_WRITE_VAR(i, minimumfeedrate);
  EEPROM_WRITE_VAR(i, mintravelfeedrate);
}

void Config_PrintSettings()
{
  // Allways have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
  SERIAL_PROTOCOLLNPGM("Steps per unit (steps/mm)");
  SERIAL_PROTOCOLPGM_PAIR("X: ", axis_steps_per_unit[0]);
  SERIAL_PROTOCOLPGM_PAIR(" Y: ", axis_steps_per_unit[1]);
  SERIAL_PROTOCOLPGM_PAIR(" Z: ", axis_steps_per_unit[2]);
  SERIAL_PROTOCOLPGM_PAIR(" E: ", axis_steps_per_unit[3]);
  SERIAL_PROTOCOL_END;

  SERIAL_PROTOCOLPGM_PAIR("Minimum feedrate (mm/s): ", minimumfeedrate);
  SERIAL_PROTOCOL_END;

  SERIAL_PROTOCOLPGM_PAIR("Minimum travel feedrate (mm/s): ", mintravelfeedrate);
  SERIAL_PROTOCOL_END;
}

void Config_RetrieveSettings()
{
  int i = EEPROM_OFFSET;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver);
  if (strncmp(ver, stored_ver, 3) == 0)
  {
    EEPROM_READ_VAR(i, axis_steps_per_unit);
    EEPROM_READ_VAR(i, minimumfeedrate);
    EEPROM_READ_VAR(i, mintravelfeedrate);
    SERIAL_PROTOCOLLNPGM(MSG_STORED_SETTINGS_LOADED);
  }
  else
  {
    Config_ResetDefault();
    SERIAL_PROTOCOLLNPGM(MSG_DEFAULT_SETTINGS_LOADED);
  }
  Config_PrintSettings();
}

void Config_ResetDefault()
{
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  for (short i = 0; i < 4; i++)
  {
    axis_steps_per_unit[i] = tmp1[i];
  }
  minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
  mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;

}

