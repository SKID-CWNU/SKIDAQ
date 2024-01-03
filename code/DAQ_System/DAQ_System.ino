// Raspberry Pi Pico based Steering Wheel Main Circuit code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //

#include <Arduino.h>
#include <SPI.h>
#include <DHT.h>

const int SPI_CS_PIN = 22;
const int CAN_INT_PIN = 20;

#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
   
    Serial.begin(115200);
    while (!Serial)
    {
    };

    while (CAN_OK != CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("CAN init fail, retry...");
        delay(100);
    }
    Serial.println("CAN init ok!");
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
    unsigned char stmp[8] = {char(h), char(t), 0, 0, 0, 0, 0, 0};

    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    stmp[7] = stmp[7] + 1;
    if (stmp[7] == 100)
    {
        stmp[7] = 0;
        stmp[6] = stmp[6] + 1;

        if (stmp[6] == 100)
        {
            stmp[6] = 0;
            stmp[5] = stmp[5] + 1;
        }
    }

    CAN.sendMsgBuf(0x00, 0, 8, stmp);
    delay(100); // send data per 100ms
}