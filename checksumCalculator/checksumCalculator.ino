/*===== Compiler Definitions =====*/
#define BAUDRATE 115200
/*===== =====*/
/*===== Private Variables =====*/
char serial_char;
char serial_char_last;
byte checksum = 0;

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
      Serial.print("*");
      Serial.println(checksum);
      Serial.flush();
    }
    else
    {
      Serial.print(serial_char);
      checksum ^= serial_char;
      serial_char_last = serial_char;
    }
  }
}
