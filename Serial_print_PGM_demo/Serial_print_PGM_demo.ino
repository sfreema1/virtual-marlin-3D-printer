// Serial printing from PGM demo
// use test messagemagic[] example to study effect of method
// of communication on allocation of bytes

// All test listed below define messagemagic[], setup() with Serial.begin(), serialprintPGM()
// as baseline

// Baseline:                    1960 bytes of program space | 182 bytes of dynamic memory
// Serial.print(""):            2018 bytes of program space | 184 bytes of dynamic memory
// Serial.println(""):          2098 bytes of program space | 184 bytes of dynamic memory
// Serial.write(""):            1990 bytes of program space | 184 bytes of dynamic memory
// serialprintPGM(""):          2018 bytes of program space | 184 bytes of dynamic memory
// foo_serialprintPGM(""):      2018 bytes of program space | 184 bytes of dynamic memory
// blah_serialprintPGM(""):     2018 bytes of program space | 184 bytes of dynamic memory
// SERIAL_PROTOCOLPGM(""):      2018 bytes of program space | 182 bytes of dynamic memory
// BLAH_SERIAL_PROTOCOLPGM(""): 2018 bytes of program space | 182 bytes of dynamic memory

// Serial.print(MESSAGEMAGIC);            2048 bytes of program space | 214 bytes of dynamic memory
// Serial.println(MESSAGEMAGIC);          2128 bytes of program space | 214 bytes of dynamic memory
// Serial.write(MESSAGEMAGIC);            2020 bytes of program space | 214 bytes of dynamic memory
// serialprintPGM(MESSAGEMAGIC);          2048 bytes of program space | 214 bytes of dynamic memory
// foo_serialprintPGM(MESSAGEMAGIC);      2048 bytes of program space | 214 bytes of dynamic memory
// blah_serialprintPGM(MESSAGEMAGIC);     2048 bytes of program space | 214 bytes of dynamic memory
// SERIAL_PROTOCOLPGM(MESSAGEMAGIC):      2048 bytes of program space | 182 bytes of dynamic memory
// BLAH_SERIAL_PROTOCOLPGM(MESSAGEMAGIC): 2048 bytes of program space | 182 bytes of dynamic memory

// Serial.print(messagemagic);            2048 bytes of program space | 214 bytes of dynamic memory
// Serial.println(messagemagic);          2128 bytes of program space | 214 bytes of dynamic memory
// Serial.write(messagemagic);            2020 bytes of program space | 214 bytes of dynamic memory
// serialprintPGM(messagemagic);          2048 bytes of program space | 214 bytes of dynamic memory
// foo_serialprintPGM(messagemagic);      2048 bytes of program space | 214 bytes of dynamic memory
// blah_serialprintPGM(messagemagic);     2048 bytes of program space | 214 bytes of dynamic memory
// SERIAL_PROTOCOLPGM(messagemagic):      ???? bytes of program space | ??? bytes of dynamic memory **Compilation failed**
// BLAH_SERIAL_PROTOCOLPGM(messagemagic): ???? bytes of program space | ??? bytes of dynamic memory **Compilation failed**

#define  FORCE_INLINE __attribute__((always_inline)) inline
#define MESSAGEMAGIC "Test print a very long message"
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define BLAH_SERIAL_PROTOCOLPGM(x) (blah_serialprintPGM(PSTR(x)))

const char messagemagic[] = "Test print a very long message";

// Things to write to serial from PROGMEM. Saves 400 to 2k of RAM.
void serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    Serial.write(ch);
    ch = pgm_read_byte(++str);
  }
}

// Things to write to serial from PROGMEM. Saves 400 to 2k of RAM.
void foo_serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    Serial.write(ch);
    ch = pgm_read_byte(++str);
  }
}

// Things to write to serial from PROGMEM. Saves 400 to 2k of RAM.
FORCE_INLINE void blah_serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    Serial.write(ch);
    ch = pgm_read_byte(++str);
  }
}

void setup()
{
  Serial.begin(115200);
  //Test serial methods with empty string ""
  //Serial.print("");
  //Serial.println("");
  //Serial.write("");
  //serialprintPGM("");
  //foo_serialprintPGM("");
  //blah_serialprintPGM("");
  //SERIAL_PROTOCOLPGM("");
  //BLAH_SERIAL_PROTOCOLPGM("");

  //Test serial method with MESSAGEMAGIC
  //Serial.print(MESSAGEMAGIC);
  //Serial.println(MESSAGEMAGIC);
  //Serial.write(MESSAGEMAGIC);
  //serialprintPGM(MESSAGEMAGIC);
  //foo_serialprintPGM(MESSAGEMAGIC);
  //blah_serialprintPGM(MESSAGEMAGIC);
  //SERIAL_PROTOCOLPGM(MESSAGEMAGIC);
  //BLAH_SERIAL_PROTOCOLPGM(MESSAGEMAGIC);


  //Test serial method with messagemagic[]
  //Serial.print(messagemagic);
  //Serial.println(messagemagic);
  //Serial.write(messagemagic);
  //serialprintPGM(messagemagic);
  //foo_serialprintPGM(messagemagic);
  //blah_serialprintPGM(messagemagic);
  //SERIAL_PROTOCOLPGM(messagemagic);
  //BLAH_SERIAL_PROTOCOLPGM(messagemagic);
}

void loop()
{
}
