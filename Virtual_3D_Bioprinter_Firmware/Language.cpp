#include "Language.h"
#include "Arduino.h"

/*===== Routines =====*/
void serial_echopair_P(const char *s_P, float v)
{ serialprintPGM(s_P); Serial.print(v); }
void serial_echopair_P(const char *s_P, double v)
{ serialprintPGM(s_P); Serial.print(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
{ serialprintPGM(s_P); Serial.print(v); }
