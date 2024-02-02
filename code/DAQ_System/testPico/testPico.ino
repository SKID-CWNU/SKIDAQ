

// Crank Position Sensor simulator
const byte OutputPin = 10;
const byte ControlPin = A0;

int TotalTeeth = 3;
int MissingTeeth = 0;
int TeethBetweenMissingTeeth = 0;

const unsigned long MicrosecondsPerMinute = 1000000ul * 60;

void setup()
{
  Serial.begin(9600);
  pinMode(OutputPin, OUTPUT);
}

void Tooth(int duration)
{
  digitalWrite(OutputPin, HIGH);
  delayMicroseconds(duration);
  digitalWrite(OutputPin, LOW);
  delayMicroseconds(duration);
}

void ToothMissing(int duration)
{
  // two delays for both halves of a missing pulse
  delayMicroseconds(duration);
  delayMicroseconds(duration);
}

void loop()
{
  int RPM = map(analogRead(ControlPin), 102, 922, 1550, 3600);
Serial.print(RPM);
  unsigned long pulsesPerMinute = RPM * TotalTeeth;

  unsigned long microsecondsPerPulse = MicrosecondsPerMinute / pulsesPerMinute;
  unsigned long microsecondsPerHalfPulse = microsecondsPerPulse / 2;


  for (int i = 0; i < TotalTeeth - MissingTeeth - (TeethBetweenMissingTeeth * (MissingTeeth - 1)); i++)
  {
    Tooth(microsecondsPerHalfPulse);
  }

  for (int i = 0; i < MissingTeeth; i++)
  {
    ToothMissing(microsecondsPerHalfPulse);

    // Between pairs of missing teeth, insert this many teeth
    if (i < MissingTeeth - 1)
    {
      for (int j = 0; j < TeethBetweenMissingTeeth; j++)
      {
        Tooth(microsecondsPerHalfPulse);
      }
    }
  }
  delay(100);
}