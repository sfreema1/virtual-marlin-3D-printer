/***** CHECKSUMMING *****/
/*
This sketch will show you how the checksumming of gcode works in the Marlin
firmware.

Please reset before every use (bug needs to be fixed)!!
*/
/**********/
/*===== Compiler Definitions =====*/
#define BAUDRATE 115200
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define COMMAND_RECEIVED "The command you sent was "
#define CHECKSUM_INITIATED "A checksum has been prompted"
#define INITIAL_CHECKSUM "The initial checksum value in binary was "
#define CHAR_SENT "The char sent was a "
/*==========*/
/*===== Private Variables =====*/
static char serial_char;
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindw = 0;
static int serial_count = 0;
static char *strchr_pointer;
/*==========*/

void setup()
{
  Serial.begin(BAUDRATE);
}

void loop()
{
  while (Serial.available() > 0)
  {
    serial_char = Serial.read();
    if (serial_char == '\n' || serial_char == '\r')
    {
      Serial.print(COMMAND_RECEIVED);
      Serial.println(cmdbuffer[bufindw]);

      if (strchr(cmdbuffer[bufindw], '*') != NULL)
      {
        byte checksum = 0;
        byte checksum_last = 0;
        int answer;
        byte count = 0;

        Serial.println(CHECKSUM_INITIATED);
        Serial.print(INITIAL_CHECKSUM);
        Serial.println(checksum, BIN);

        while (cmdbuffer[bufindw][count] != '*')
        {
          Serial.print(CHAR_SENT);
          Serial.print(cmdbuffer[bufindw][count]);
          Serial.print(", which in binary is ");
          Serial.println(cmdbuffer[bufindw][count], BIN);
          Serial.print(checksum_last, BIN);
          Serial.print(" ^ ");
          Serial.print(cmdbuffer[bufindw][count], BIN);
          Serial.print(" = ");

          checksum = checksum ^ cmdbuffer[bufindw][count++];

          Serial.println(checksum, BIN);

          checksum_last = checksum;
        }

        Serial.print("The final checksum valume in binary is ");
        Serial.print(checksum, BIN);
        Serial.print(", which in decimal representation is ");
        Serial.println(checksum, DEC);

        strchr_pointer = strchr(cmdbuffer[bufindw], '*');
        answer = (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL));
        Serial.print("The correct checksum should be ");
        Serial.println(answer, DEC);
        if ( answer != checksum)
        {
          Serial.println("Error: Checksum mismatch");
        }
        else
        {
          Serial.println("Success! Checksums match");
        }
        serial_count = 0;
      }
    }
    else
    {
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }

  }
}
