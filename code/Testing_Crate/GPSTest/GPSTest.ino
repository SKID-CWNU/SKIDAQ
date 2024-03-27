#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
int RXPin = 14;
int TXPin = 15;

// Default baud of NEO-6M is 9600
int GPSBaud = 9600;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
    // Start the Arduino hardware serial port at 9600 baud
    Serial.begin(9600);
    delay(1000);
    // Start the software serial port at the GPS's default baud
    gpsSerial.begin(GPSBaud);
    delay(100);
}

void loop()
{
    // Displays information when new sentence is available.
    while (gpsSerial.available() > 0)
    {
        Serial.write(gpsSerial.read());
    }
    while (Serial.available() > 0)
    {
        gpsSerial.write(Serial.read());
    }
}