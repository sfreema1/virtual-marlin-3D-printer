#include "Configuration.h"
#include "Buffer.h"


void setup()
{
  Serial.begin(BAUDRATE);  
}

void loop()
{
  if(buflen < (BUFSIZE - 1)) get_command(); // Retrieve command from serial
  
  if(buflen)
  {
    buflen = (buflen - 1);
    bufindr = (bufindr + 1);
  }
}
