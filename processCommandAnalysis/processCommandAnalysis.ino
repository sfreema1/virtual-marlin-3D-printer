/***** NOTES *****/
/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Checksumming using XORing of all bits up until the asterisk (*).
A correct checksumming
XOR-ing
If byte A = 1010 and byte B = 1001, then A^B =:
A: 1---0---1---0
B: 1---0---0---1
   =============
C: 0---0---1---1
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
STRUCTURE OF THE GET_PROCESS() ROUTINE

The routine will first check to see if the incoming char is indicative of the end of a command. This is done by checking
for newline feeds or return carriages. It will also check for colons (:) or if it has reached the max length available
to a command.

If none of these conditions are met, then it will first check to see whether the incoming char is a semicolon (;), which
indicates it and what follows it will be a comment. It will also independently check whether comment mode has already
been activated (comment_mode == TRUE). If it has not been activated, get_process() will store the char in the command buffer
(cmdbuffer). The cmdbuffer is built up as each new char arrives, and the first space in
*/
/***** END OF NOTES *****/

/*===== Compiler Definitions =====*/
/*----- Printer Communication ----*/
#define BAUDRATE 115200
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
/*-----Messages----*/
#define MSG_START "Welcome to the Virtual Printer!!"
#define MSG_STR_TERMINATE "Command portion of line has ended. Terminating string."
#define MSG_ENTER_COMMAND "Please enter a G-code command to see what happens."
#define MSG_EMPTY_COMMAND "No command in line. Exiting get_command()"
#define MSG_NON_COMMENT_CHAR "Current char is not part of a comment."
/*==========*/
/*===== Private Variables =====*/
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0; // buffer index for reading
static int bufindw = 0; // buffer index for writing
static int buflen = 0; // buffer length
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
bool Stopped = false;
static unsigned long previous_millis_cmd = 0;
static int full_buffer_alert = 0;
/*==========*/

/*===== MAIN SETUP() & LOOP() =====*/

void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println(MSG_START);
  Serial.println(MSG_ENTER_COMMAND);
  Serial.println("You will be alerted when the buffer is full.");
  Serial.println("");
}

void loop()
{
  if (buflen < (BUFSIZE - 1))
  {
    get_command();
  }
  else
  {
    if (full_buffer_alert == 0)
    {
      Serial.println("Buffer is full. No more commands can be sent.");
      full_buffer_alert += 1;
    }
  }

  if (buflen)
  {
    process_commands();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
  }
}

/*===== =====*/


/*===== PRINTER ROUTINES =====*/
void get_command()
{
  // While loops checks for incoming char of a command but only while the buffer can hold another command
  while ( Serial.available() > 0  && buflen < BUFSIZE)
  {
    serial_char = Serial.read();

    // If 1
    if (serial_char == '\n' ||
        serial_char == '\r' ||
        (serial_char == ':' && comment_mode == false) ||
        serial_count >= (MAX_CMD_SIZE - 1) )
    {
      // If 1a
      if (!serial_count) //if empty line
      {
        Serial.println(MSG_EMPTY_COMMAND);
        comment_mode = false; //for new command
        Serial.println("");
        return;
      }
      Serial.println(MSG_STR_TERMINATE);
      cmdbuffer[bufindw][serial_count] = 0; //terminate string

      // If 1b
      if (!comment_mode)
      {
        comment_mode = false; //for next line
        Serial.println(MSG_NON_COMMENT_CHAR);
        Serial.println("Checking for line number ...");

        // If 1ba
        if (strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          Serial.println("An line number has been detected in the command.");
          Serial.println("Checking line number continuity...");

          // If 1baa
          if (gcode_N != gcode_LastN + 1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) )
          {
            Serial.print("Error: ");
            Serial.print("Line number is not last line number plus one, Last line: ");
            Serial.println(gcode_N);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          // End of 1baa

          Serial.println("Line number check complete: Success !");
          Serial.println("Looking for checksum in line ...");
          // If 1bab
          if (strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            Serial.println("A checksum has been prompted in the command. Calculating checksum ...");
            byte checksum = 0;
            byte count = 0;
            while (cmdbuffer[bufindw][count] != '*') checksum = checksum ^ cmdbuffer[bufindw][count++];
            Serial.print("Calculated checksum in binary is ");
            Serial.print(checksum, BIN);
            Serial.print(" and in decimal represation is ");
            Serial.println(checksum, DEC);
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
            Serial.print("Comparing calculated checksum with expected checksum value of ");
            Serial.println((int)strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL));

            // If 1baba
            if ( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
            {
              Serial.print("Error: ");
              Serial.print("Checksum mismatch. Calculated checksum does match expected checksum value. Last line: ");
              Serial.println(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            Serial.println("Checksum match! Success!");
            // End of 1baba
            // if no errors, continue parsing
          }
          // Else 1bab
          else
          {
            Serial.print("Error: ");
            Serial.print("No checksum with line number. Checksum is needed if line number is used. Last Line: ");
            Serial.println(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          // End of 1bab
          Serial.println("Success! Both line number and checksum have been verified.");

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        // Else 1ba
        else  // if we don't receive 'N' but still see '*'
        {
          Serial.println("No line number has been detected.");
          Serial.println("Checking for checksum ...");
          // If 1bac
          if ((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            Serial.print("Error: ");
            Serial.print("Checksum deteched without line number. No Line number with");
            Serial.println(gcode_LastN);
            serial_count = 0;
            return;
          }
          Serial.println("Neither line number nor checksum detected.");
        }
        // End of 1ba

        // If 1bb
        Serial.println("Checking for G-commands ....");
        if ((strchr(cmdbuffer[bufindw], 'G') != NULL))
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          Serial.print("G-command detected: G");
          Serial.println((int)strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL));
          switch ((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
          {
            case 0:
            case 1:
            case 2:
            case 3:
              // If bba
              Serial.println("Movement G[0-3] command detected.");
              Serial.println("Checking if the printer is stopped. Will return OK if printer is not stopped and movement is possible.");
              if (Stopped == false) // If printer is stopped by an error the G[0-3] codes are ignored.
              {
                Serial.println("OK");
              }
              // Else bba
              else
              {
                Serial.println("Printer stopped due to errors. Fix the error and use M999 to restart.");
              }
              // End of If bba
              break;
            default:
              Serial.println("Non-movement G-command.");
              break;
          }
        }
        //If command was e-stop process now
        //End of If 1bb

        // If 1bc
        Serial.println("Checking for emergency stop request M-112 ...");
        if (strcmp(cmdbuffer[bufindw], "M112") == 0) kill();
        // End of 1bc
        Serial.println("No emergency stop requested.");


        bufindw = (bufindw + 1) % BUFSIZE;
        Serial.print("Bufindw value is: ");
        Serial.println(bufindw);
        buflen += 1;
        Serial.print("Buflen value increased to: ");
        Serial.println(buflen);
      }
      // End of 1b

      serial_count = 0; //clear buffer
      Serial.println("Serial count reset to zero.");
      Serial.println("");
    }
    // Else 1
    else
    {
      if (serial_char == ';')
      {
        Serial.println("Beginning of comment in line has been detected.");
        comment_mode = true;
      }
      if (!comment_mode)
      {
        cmdbuffer[bufindw][serial_count++] = serial_char;
        Serial.print("The char ");
        received_char_print();
        Serial.println(" has been received and store in the buffer.");

      }
      else
      {
        Serial.println("Comment char not stored.");
      }
    }
    // End of If 1
  }
}

void process_commands()
{
  unsigned long codenum; // throw away variable
  char *starpos = NULL;
  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      case 0: //G0 ->G1
      case 1:
        if (Stopped == false)
        {
          Serial.println("Getting coordinates ...");
          Serial.println("Preparing to move ...");
          return;
        }
        break;
    }
  }
  else if (code_seen('M'))
  {
  }
  else if (code_seen('T'))
  {
  }
  else
  {
    Serial.print("Echo: ");
    Serial.print("Unknown command: \"");
    Serial.print(cmdbuffer[bufindr]);
    Serial.println("\"");
  }
  ClearToSend();
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);
}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

void FlushSerialRequestResend()
{
  Serial.flush();
  Serial.print("Resend: ");
  Serial.println(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  Serial.println("OK");
}

void kill()
{
  Serial.print("Error: ");
  Serial.println("Printer halted. kill() routine called!");
  while (1) {}
}

/*==========*/

/*===== ANALYSIS METHODS =====*/


void analyzer_print()
{
  Serial.print("Current line number is ");
  Serial.println(gcode_N);
  Serial.print("Comment mode is ");
  Serial.println(comment_mode);
  Serial.print("Bufindr value is ");
  Serial.println(bufindr);
  Serial.print("Bufindw value is ");
  Serial.println(bufindw);
  Serial.print("Buflen value is ");
  Serial.println(buflen);
  Serial.println("");
}

void received_char_print()
{
  if (serial_char == '\n')
  {
    Serial.print("\n");
  }
  else if (serial_char == '\r')
  {
    Serial.print("\n");
  }
  else if (serial_char == ' ')
  {
    Serial.print("SPACE");
  }
  else
  {
    Serial.print(serial_char);
  }
}
