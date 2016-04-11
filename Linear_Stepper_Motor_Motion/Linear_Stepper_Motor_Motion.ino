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
  digitalWrite(STEP_PIN, HIGH);
  delay(STEP_INTERVAL);
  digitalWrite(STEP_PIN, LOW);
  delay(STEP_INTERVAL);
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
