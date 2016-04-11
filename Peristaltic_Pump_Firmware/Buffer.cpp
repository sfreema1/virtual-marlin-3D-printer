#include "Buffer.h"
#include "Arduino.h"
#include "Language.h"
#include "Configuration.h"

/*===== Variables =====*/
char serial_char; // variable for incoming read from serial
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE]; // command buffer
int serial_count = 0; // for tracking number of characters received
int buflen = 0; // current length of command buffer
int bufindw = 0; // buffer write index
int bufindr = 0; // buffer read index
char *strchr_pointer; // char pointer for parsing commands

/*===== Routines =====*/

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_command() // Routine to retrieve commands from serial when available
{
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read();
    if (serial_char == '\n' || serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1))
    {
      if (!serial_count) return; // empty line

      cmdbuffer[bufindw][serial_count] = 0;
      bufindw = (bufindw + 1) % BUFSIZE;
      buflen += 1;
      serial_count = 0;
      SERIAL_PROTOCOLLNPGM(MSG_COMMAND_STORED);
    }
    else
    {
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}


void process_commands() // Routine to interpret commands sent over serial
{
  unsigned long codenum; // throwaway variable
  char *starpos = NULL;

  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
    }
  }
}

