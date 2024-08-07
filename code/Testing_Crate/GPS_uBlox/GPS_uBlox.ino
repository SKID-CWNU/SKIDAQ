/*
  Reading lat, long and UTC time via UBX binary commands - no more NMEA parsing!
  By: Paul Clark and Nathan Seidle
  Using the library modifications provided by @blazczak and @geeksville

  SparkFun Electronics
  Date: June 16th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a Ublox module for its lat/long/altitude. We also
  turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic
  dramatically.

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library_Series_6_7.h"
SFE_UBLOX_GPS myGPS;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2); // RX, TX. Pin 10 on Uno goes to TX pin on GPS module.

long lastTime = 0; // Simple local timer. Limits amount of I2C traffic to Ublox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  // Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  // Loop until we're in sync and then ensure it's at 38400 baud.
  do
  {
    Serial.println("GPS: trying 38400 baud");
    mySerial.begin(38400);
    if (myGPS.begin(mySerial) == true)
      break;

    delay(100);
    Serial.println("GPS: trying 9600 baud");
    mySerial.begin(9600);
    if (myGPS.begin(mySerial) == true)
    {
      Serial.println("GPS: connected at 9600 baud, switching to 38400");
      myGPS.setSerialRate(38400);
      delay(100);
    }
    else
    {
      // myGPS.factoryReset();
      delay(2000); // Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GPS serial connected");

  myGPS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();          // Save the current settings to flash and BBR
}

void loop()
{
  // Query module only every second. Doing it more often will just cause I2C traffic.
  // The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); // Update the timer

    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.print(F(" Time: "));

    byte Hour = myGPS.getHour();
    if (Hour < 10)
    {
      Serial.print(F("0"));
    }
    Serial.print(Hour);
    Serial.print(F(":"));

    byte Minute = myGPS.getMinute();
    if (Minute < 10)
    {
      Serial.print(F("0"));
    }
    Serial.print(Minute);
    Serial.print(F(":"));

    byte Second = myGPS.getSecond();
    if (Second < 10)
    {
      Serial.print(F("0"));
    }
    Serial.print(Second);

    Serial.println();
  }
}
