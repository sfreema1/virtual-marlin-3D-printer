#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"

void Config_ResetDefault();

void Config_PrintSettings();

/* EEPROM SETTINGS */
void Config_StoreSettings();
void Config_RetrieveSettings();

#endif//CONFIG_STORE_H
