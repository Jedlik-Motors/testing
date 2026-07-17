//==============================
// NEMA23 + HSS57 Driver Test
// Arduino Mega 2560
//==============================

#define PUL_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4

// Delay between pulses
// Larger = slower
// Smaller = faster
int pulseDelay = 15;   // microseconds

void setup()
{
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  // Enable Driver
  digitalWrite(ENA_PIN, LOW);     // Change to HIGH if your driver uses active HIGH enable
}

void loop()
{
  //==========================
  // Clockwise Rotation
  //==========================
  digitalWrite(DIR_PIN, HIGH);

  for(int i = 0; i < 30000; i++)
  {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(pulseDelay);

    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(pulseDelay);
  }

  delay(1000);

  //==========================
  // Counter Clockwise
  //==========================
  digitalWrite(DIR_PIN, LOW);

  for(int i = 0; i < 30000; i++)
  {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(pulseDelay);

    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(pulseDelay);
  }

  delay(1000);
}