#include <CAN.h>
#include <SoftwareSerial.h>

SoftwareSerial sensorSerial(22, 21); // RX, TX

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB port only
          // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3))
    {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
    // set the data rate for the SoftwareSerial port
    sensorSerial.begin(9600);
    sensorSerial.println("Steering Wheel Display Connected.");
}

void loop()
{ // run over and over
    // try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize || CAN.packetId() != -1)
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