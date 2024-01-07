// Raspberry Pi Pico based Steering Wheel Main Circuit code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //

#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>
#include <DHT.h>

#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

void setup()
{

    Serial.begin(115200);
    while (!Serial)
    {
    };
    // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3))
    {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
    dht.begin();
}

void loop()
{
    int h = dht.readHumidity();    // Gets Humidity value
    int t = dht.readTemperature(); // Gets Temperature value

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    Serial.print("Sending DHT packet ... ");

    CAN.beginPacket(0x12);
    CAN.write("d");
    CAN.write(h);
    CAN.write(t);
    CAN.write();
    CAN.write();
    CAN.endPacket();

    Serial.println("done");

    delay(100); // send data per 100ms
}