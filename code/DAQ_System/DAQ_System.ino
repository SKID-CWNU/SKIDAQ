// ——————————————————————————————————————————————————————————————————————————————//
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                Solenoid Valve Controlled QuickShift Interface                 //
//                             Author: Lim Chae Won                              //
// ——————————————————————————————————————————————————————————————————————————————//

#include "DHT.h"
#include <SoftwareSerial.h>
#include "mcp_can.h"
#include "mcp2515_can.h"
#include <SPI.h>
#include <Arduino.h>

// ——————————————————————————————————————————————————————————————————————————————
//    DHT Library Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define DHTPIN 10     // DHT Signal Pin
#define DHTTYPE DHT22 // DHT Sensor type: DHT22
DHT dht(DHTPIN, DHTTYPE);

// ——————————————————————————————————————————————————————————————————————————————
//    CAN Interface Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define INT32U unsigned long int
mcp2515_can CAN(17); // Set CS to pin 17
const int CAN_INT_PIN = 11;

INT32U canId = 0x000;

unsigned char len = 0;
unsigned char buf[8];
char str[20];

String BuildMessage = "";
int MSGIdentifier = 0;
// ——————————————————————————————————————————————————————————————————————————————
//    Other Sensors Initalization
// ——————————————————————————————————————————————————————————————————————————————
int picoTemp = analogReadTemp();

// ——————————————————————————————————————————————————————————————————————————————
//    Main Program Setup
// ——————————————————————————————————————————————————————————————————————————————
void setup()
{
    Serial.begin(115200);
    dht.begin(); // DHT Sensor Begin
START_INIT:

    if (CAN_OK == CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT; //wait 100ms and try Initializing again
    }
}
// ——————————————————————————————————————————————————————————————————————————————
//    Main(Loop) Area
// ——————————————————————————————————————————————————————————————————————————————
void loop()
{
    // DHT Area
    int h = dht.readHumidity();    // Gets Humidity value
    int t = dht.readTemperature(); // Gets Temperature value

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
        Serial.println("DHT Error: Failed to read from DHT sensor!");
        return;
    }
    // Printing Values for Monitoring
    // Serial.print("Humidity: ");
    // Serial.print(h);
    // Serial.print("%  Temperature: ");
    // Serial.print(t);
    // ——————————————————————————————————————————————————————————————————————————————

    // CAN Area
    char rndCoolantTemp = random(1, 200);
    char rndRPM = random(1, 55);
    char rndSpeed = random(0, 255);
    char rndIAT = random(0, 255);
    char rndMAF = random(0, 255);
    char ControlAirTemp = h;
    char OBTemp = picoTemp;

    // GENERAL ROUTINE
    unsigned char SupportedPID[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    unsigned char MilCleared[7] = {4, 65, 63, 34, 224, 185, 147};

    // SENSORS
    unsigned char CoolantTemp[7] = {4, 65, 5, rndCoolantTemp, 0, 185, 147};
    unsigned char rpm[7] = {4, 65, 12, rndRPM, 224, 185, 147};
    unsigned char vspeed[7] = {4, 65, 13, rndSpeed, 224, 185, 147};
    unsigned char IATSensor[7] = {4, 65, 15, rndIAT, 0, 185, 147};
    unsigned char MAFSensor[7] = {4, 65, 16, rndMAF, 0, 185, 147};
    unsigned char AmbientAirTemp[7] = {4, 65, 70, ControlAirTemp, 0, 185, 147};
    unsigned char CAT1Temp[7] = {4, 65, 60, OBTemp, 224, 185, 147};
    unsigned char CAT2Temp[7] = {4, 65, 61, OBTemp, 224, 185, 147};
    unsigned char CAT3Temp[7] = {4, 65, 62, OBTemp, 224, 185, 147};
    unsigned char CAT4Temp[7] = {4, 65, 63, OBTemp, 224, 185, 147};

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {

        CAN.readMsgBuf(&len, buf);
        canId = CAN.getCanId();
        Serial.print("<");
        Serial.print(canId);
        Serial.print(",");

        for (int i = 0; i < len; i++)
        {
            BuildMessage = BuildMessage + buf[i] + ",";
        }
        Serial.println(BuildMessage);

        // Check wich message was received.
        if (BuildMessage == "2,1,0,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID);
        }
        if (BuildMessage == "2,1,1,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, MilCleared);
        }

        // SEND SENSOR STATUSES
        if (BuildMessage == "2,1,5,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, CoolantTemp);
        }
        if (BuildMessage == "2,1,12,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, rpm);
        }
        if (BuildMessage == "2,1,13,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, vspeed);
        }
        if (BuildMessage == "2,1,15,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, IATSensor);
        }
        if (BuildMessage == "2,1,16,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, MAFSensor);
        }
        if (BuildMessage == "2,1,70,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, AmbientAirTemp);
        }
        if (BuildMessage == "2,1,60,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, CAT1Temp);
        }
        if (BuildMessage == "2,1,61,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, CAT2Temp);
        }
        if (BuildMessage == "2,1,62,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, CAT3Temp);
        }
        if (BuildMessage == "2,1,63,0,0,0,0,0,")
        {
            CAN.sendMsgBuf(0x7E8, 0, 7, CAT4Temp);
        }
        BuildMessage = "";
    }
    delay(100);
}

// ——————————————————————————————————————————————————————————————————————————————
//   OBD-II PIDs Configuration - https://en.wikipedia.org/wiki/OBD-II_PIDs
// ——————————————————————————————————————————————————————————————————————————————

// Extra Configurations
#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif
