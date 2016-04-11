#ifndef CONFIGURATIONSTORE_H
#define CONFIGURATIONSTORE_H

#include "Configuration.h"

/*===== Routines =====*/
void Config_ResetDefault();
void Config_PrintSettings();
/* EEPROM SETTINGS */
void Config_StoreSettings();
void Config_RetrieveSettings();

#endif // CONFIGURATIONSTORE_H
