

// Raspberry Pi Pico based Steering Wheel Main Circuit code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //
#include <CAN.h>
#include <CANController.h>
#include <ESP32SJA1000.h>
#include <MCP2515.h>
#include <SPI.h>
#include "DHT.h"
#include <SoftwareSerial.h>

#define DHTPIN 10
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
    Serial.begin(115200);
    CAN.setPins(17, 20);
    Serial.println("CAN Sender");
    // start the CAN bus at 500 kbps
    CAN.begin(500E3);
    if (!CAN.begin(500E3))
    {
        delay(1000);
        Serial.println("Starting CAN failed!");
        return 1;
    }

    dht.begin();
    Serial.println("DAQ System Init Success!");
    delay(5000);
}

void loop()
{
    int h = dht.readHumidity();    // Gets Humidity value
    int t = dht.readTemperature(); // Gets Temperature value

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print("%  Temperature: ");
    Serial.print(t);
    delay(1000);
    Serial.println("Sending DHT packet ... ");

    CAN.beginPacket(0x12);
    CAN.write('d');
    CAN.write(h);
    CAN.write(t);
    CAN.endPacket();

    Serial.println("done");

    delay(1000); // send data per 100ms
}