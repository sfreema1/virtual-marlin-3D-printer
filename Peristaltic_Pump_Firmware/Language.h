#ifndef LANGUAGE_H
#define LANGUAGE_H

#define FORCE_INLINE __attribute__((always_incline)) inline

#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),Serial.write('\n'))

/*===== Messages =====*/
#define MSG_INSTRUCTION "Enter Code I1 for information on commands."
#define MSG_COMMAND_STORED "Command successfully stored."

// things to write to serial from Program Memory. Saves 400 to 2k of RAM
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while(ch)
  {
    Serial.write(ch);
    ch = pgm_read_byte(++str);
  }
}

#endif // LANGUAGE_H
