#include "Configuration.h"
#include "Buffer.h"
#include "Language.h"
#include "ConfigurationStore.h"


void setup()
{
  Serial.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM(MSG_WELCOME);
  Config_RetrieveSettings();
  SERIAL_PROTOCOL_END;
}

void loop()
{
  if (buflen < (BUFSIZE - 1)) get_command(); // Retrieve command from serial

  if (buflen)
  {
    process_commands();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
}
