#define STEP_PIN 8
#define DIR_PIN 7
#define MS1_PIN 11
#define MS2_PIN 10
#define MS3_PIN 9
#define ENABLE_PIN 12
// Stepping Configuration: Comment out all except the mode you want to test
#define FULL_STEP
// #define HALF_STEP
// #define QUARTER_STEP
// #define EIGHTH_STEP
// #define SIXTEENTH_STEP

// Command Storage Variables
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

char serial_char;
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
int serial_count = 0;
int buflen = 0;
int bufindw = 0;
int bufindr = 0;
char *strchr_pointer;

// Stepper Motor Motion Variables
#define STEP_INTERVAL 1 // delay between HIGH and LOW for step motion in ms
int counts;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello!");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

#ifdef FULL_STEP //STEP MODE
  full_step_mode();
#elif defined(HALF_STEP)
  half_step_mode();
#elif defined(QUARTER_STEP)
  quarter_step_mode();
#elif defined(EIGHTH_STEP)
  eighth_step_mode();
#elif defined(SIXTEENTH_STEP)
  sixteeth_step_mode();
#endif // STEP_MODE
}

void loop()
{
  if (buflen < (BUFSIZE - 1)) get_command(); // Retrieve command from serial

  if (buflen)
  {
    process_commands();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
  }
}

void get_command()
{
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read(); // Received character
    // Character Processing
    if (serial_char == '\n' || serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1))
    {
      if (!serial_count) // empty line
      {
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; // terminate string
      bufindw = (bufindw + 1) % BUFSIZE;
      buflen += 1;
      serial_count = 0; // clear buffer
    }
    else
    {
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

void process_commands()
{
  unsigned long code_num; //throwaway variable
  char *starpos = NULL;
  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      case 1:
        if (code_seen('S'))
        {
          code_num = code_value();
          Serial.println("Code seen was G1");
          Serial.print("Stepper will move "); Serial.print(code_value()); Serial.println(" steps clockwise");
          Serial.println("Executing ...");
          stepper_move_clockwise(code_num);
          Serial.println("Action complete");
        }
        return;
      case 2:
        if (code_seen('S'))
        {
          code_num = code_value();
          Serial.println("Code seen was G2");
          Serial.print("Stepper will move "); Serial.print(code_value()); Serial.println(" steps counterclockwise");
          Serial.println("Executing ...");
          stepper_move_counterclockwise(code_num);
          Serial.println("Action complete");
        }
        return;
      default:
        Serial.println("Unrecognized G command. Please try again.");
        return;
    }

  }
  else if (code_seen('M'))
  {
    switch ((int)code_value())
    {
      case 1:
        full_step_mode();
        return;
      case 2:
        half_step_mode();
        return;
      case 3:
        quarter_step_mode();
        return;
      case 4:
        eighth_step_mode();
        return;
      case 5:
        sixteenth_step_mode();
        return;
      default:
        Serial.println("M Command not recognize. Please try again.");
        return;

    }
  }
  else
  {
    Serial.println("Unrecognized command. Please try again.");
  }
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);
}

int code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

void stepper_move_clockwise(int counts)
{
  int steps = 0;
  while (steps < counts)
  {
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(STEP_PIN, HIGH);
    delay(STEP_INTERVAL);
    digitalWrite(STEP_PIN, LOW);
    delay(STEP_INTERVAL);
    steps++;
  }
}

void stepper_move_counterclockwise(int counts)
{
  int steps = 0;
  while (steps < counts)
  {
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(STEP_PIN, HIGH);
    delay(STEP_INTERVAL);
    digitalWrite(STEP_PIN, LOW);
    delay(STEP_INTERVAL);
    steps++;
  }
}

void full_step_mode()
{
  digitalWrite(MS1_PIN, LOW);
  digitalWrite(MS2_PIN, LOW);
  digitalWrite(MS3_PIN, LOW);
  Serial.println("Full step mode activated");
}

void half_step_mode()
{
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, LOW);
  digitalWrite(MS3_PIN, LOW);
  Serial.println("Half step mode activated");
}

void quarter_step_mode()
{
  digitalWrite(MS1_PIN, LOW);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);
  Serial.println("Quarter step mode activated");
}

void eighth_step_mode()
{
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);
  Serial.println("Eighth step mode activated");
}

void sixteenth_step_mode()
{
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, HIGH);
  Serial.println("Sixteenth step mode activated");
}
