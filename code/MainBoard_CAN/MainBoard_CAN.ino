// Raspberry Pi Pico based Steering Wheel Main Receiver code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //

#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>
#include <SoftwareSerial.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }
    // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3))
    {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
}
void loop()
{
    int packetSize = CAN.parsePacket();

    if (packetSize)
    {
        // received a packet
        Serial.print("Received ");

        if (CAN.packetExtended())
        {
            Serial.print("extended ");
        }

        if (CAN.packetRtr())
        {
            // Remote transmission request, packet contains no data
            Serial.print("RTR ");
        }

        Serial.print("packet with id 0x");
        Serial.print(CAN.packetId(), HEX);

        if (CAN.packetRtr())
        {
            Serial.print(" and requested length ");
            Serial.println(CAN.packetDlc());
        }
        else
        {
            Serial.print(" and length ");
            Serial.println(packetSize);

            // only print packet data for non-RTR packets
            while (CAN.available())
            {
                Serial.print((char)CAN.read());
            }
            Serial.println();
        }

        Serial.println();
    }
}