/* ——————————————————————————————————————————————————————————————————————————————
    SKIDAQ Main Code
    Raspberry Pi Pico based SKID Datalogger Module
    with Solenoid Valve Controlled Non-Clutch QuickShift Interface
    Copyright © 2024 by Lim ChaeWon

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


    Pin Summary
        GP4  - ADXL345 SDA Pin
        GP5  - ADXL345 SCL Pin
        GP9  - DynoJet Interrupt Output
        GP10 - DHT Temp/Humid Sensor
        GP12 - MOSFET Upshift
        GP13 - MOSFET Downshift
        GP14 - MOSFET DRS Actuation
        GP16 - CAN RX Pin - to SO (MISO)
        GP17 - CAN CS Pin
        GP18 - CAN SCK Pin
        GP19 - CAN TX Pin - to SI (MOSI)
        GP20 - CAN Interrupt Pin
        ADC0 - CKP(RPM) Pulse Monitor

    Ref: https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg

——————————————————————————————————————————————————————————————————————————————*/

#include <SoftwareSerial.h>
#include "DFRobot_MCP2515.h"
#include <SPI.h>
#include <Wire.h>


// ——————————————————————————————————————————————————————————————————————————————
//    CAN Interface Configuration
// ——————————————————————————————————————————————————————————————————————————————
DFRobot_MCP2515 CAN(17); // Set CS to pin 17
const int CAN_INT_PIN = 20;
// Current Firmware Version
char FW_Version[] = "Beta Rev.1";
// Incoming CAN-BUS message
long unsigned int canId = 0x000;
// This is the length of the incoming CAN-BUS message
unsigned char len = 0;
// Default reply ECU ID
#define REPLY_ID 0x7E8
// This the eight byte buffer of the incoming message data payload
unsigned char buf[8];
String canMessageRead = "";
String reply = "";
// MIL on and DTC Present
bool MIL = true;
char str[20];
int MSGIdentifier = 0;
// Stored Vechicle VIN
unsigned char vehicle_Vin[18] = "1WK58FB1111111111";
// Stored Calibration ID
unsigned char calibration_ID[18] = "FW00116MHZ1111111";
// Stored ECU Name
unsigned char ecu_Name[19] = "OPENECUSIM";
// OBD standards https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_01_PID_1C
int obd_Std = 11;
// Fuel Type Coding https://en.wikipedia.org/wiki/OBD-II_PIDs#Fuel_Type_Coding
int fuel_Type = 4;

// ——————————————————————————————————————————————————————————————————————————————
//    Adafruit Unified Sensor Configuration
// ——————————————————————————————————————————————————————————————————————————————

// ——————————————————————————————————————————————————————————————————————————————
//    Other Comp. Initalization
// ——————————————————————————————————————————————————————————————————————————————
char picoTemp = analogReadTemp(); // Pi Pico On-Board Temp Sensor
int obled = LED_BUILTIN;          // On-Board LED for Basic Error Indication
// Instantiation and pins configurations
// Pin 7 - > DIO
// Pin 6 - > CLK

// ——————————————————————————————————————————————————————————————————————————————
//    Functions Initalization
// ——————————————————————————————————————————————————————————————————————————————

void MCP2515_Init(void);

void blink(int deltime);

// ——————————————————————————————————————————————————————————————————————————————
//    Switch Module Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define DynoInt 9
#define MOSUP_PIN 12
#define MOSDOWN_PIN 13
#define DRS_PIN 14
int DIAGENB = 0; //

// ——————————————————————————————————————————————————————————————————————————————
//    Main Program Setup
// ——————————————————————————————————————————————————————————————————————————————
void setup()
{
    delay(2000);
    pinMode(obled, OUTPUT);
    pinMode(MOSUP_PIN, OUTPUT);
    pinMode(MOSDOWN_PIN, OUTPUT);
    pinMode(DRS_PIN, OUTPUT);
    pinMode(DynoInt, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    delay(100);

    MCP2515_Init();
    blink(1000);
    delay(1000);
    Serial.println("——————————————————————————————————————————————————————————————————————————————");
    Serial.println("*                    SKIDAQ           " + String(FW_Version) + "                          *");
    Serial.println("*                By ChaeWon Lim https://github.com/WonITKorea               *");
    Serial.println("*                       Based on Open-Ecu-Sim-OBD2-FW                        *");
    Serial.println("*                By Rick Spooner https://github.com/spoonieau                *");
    Serial.println("——————————————————————————————————————————————————————————————————————————————");

    delay(3000);
}

// ——————————————————————————————————————————————————————————————————————————————
//   Define ECU Supported PID's
// ——————————————————————————————————————————————————————————————————————————————

/* Define the set of PIDs for MODE01 you wish you ECU to support.  For more information, see:
// https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00
//
// PID 0x01 (1) - Monitor status since DTCs cleared. (Includes malfunction indicator lamp (MIL) status and number of DTCs.)
// |   PID 0x05 (05) - Engine Coolant Temperature
// |   |      PID 0x0C (12) - Engine RPM
// |   |      |PID 0x0D (13) - Vehicle speed
// |   |      ||PID 0x0E (14) - Timing advance
// |   |      |||PID 0x0F (15) - Intake air temperature
// |   |      ||||PID 0x10 (16) - MAF Air Flow Rate
// |   |      |||||            PID 0x1C (28) - OBD standards this vehicle conforms to
// |   |      |||||            |                              PID 0x51 (58) - Fuel Type
// |   |      |||||            |                              |
// v   V      VVVVV            V                              v
// 10001000000111110000:000000010000000000000:0000000000000000100
// Converted to hex, that is the following four byte value binary to hex
// 0x881F0000 0x00 PID 01 -20
// 0x02000000 0x20 PID 21 - 40
// 0x04000000 0x40 PID 41 - 60

// Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
// we determined above (0x081F0000):

//                               0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
//                                |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
//                                |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
//                                |     |     |    0x88 - The first of four bytes of the Supported PIDS value
//                                |     |     |     |    0x1F - The second of four bytes of the Supported PIDS value
//                                |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
//                                |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
//                                |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
//                                |     |     |     |     |      |    |     |
//                                V     V     V     V     V      V    V     V*/
byte mode1Supported0x00PID[8] = {0x06, 0x41, 0x00, 0x88, 0x1F, 0x00, 0x00, 0x00};
byte mode1Supported0x20PID[8] = {0x06, 0x41, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00};
byte mode1Supported0x40PID[8] = {0x06, 0x41, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00};

/* Define the set of PIDs for MODE09 you wish you ECU to support.
// As per the information on bitwise encoded PIDs (https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00)
// Our supported PID value is:
//
//  PID 0x02 - Vehicle Identification Number (VIN)
//  | PID 0x04 (04) - Calibration ID
//  | |     PID 0x0C (12) - ECU NAME
//  | |     |
//  V V     V
// 01010000010  // Converted to hex, that is the following four byte value binary to hex
// 0x28200000 0x00 PID 01-11

// Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
// we determined above (0x28200000):

//                               0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
//                                |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
//                                |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
//                                |     |     |    0x28 - The first of four bytes of the Supported PIDS value
//                                |     |     |     |    0x20 - The second of four bytes of the Supported PIDS value
//                                |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
//                                |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
//                                |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
//                                |     |     |     |     |      |    |     |
//                                V     V     V     V     V      V    V     V*/
byte mode9Supported0x00PID[8] = {0x06, 0x49, 0x00, 0x28, 0x28, 0x00, 0x00, 0x00};

// ——————————————————————————————————————————————————————————————————————————————
//    Main(Loop) Area
// ——————————————————————————————————————————————————————————————————————————————
void loop()
{


    ////Build setting return msg
    byte obd_Std_Msg[8] = {4, 65, 0x1C, (byte)(obd_Std)};
    byte fuel_Type_Msg[8] = {4, 65, 0x51, (byte)(fuel_Type)};

    // Set to random until measuring Actual value
    unsigned int engine_Coolant_Temperature = random(1, 200);
    int rpm = random(1, 55);
    int vehicle_Speed = random(0, 255);
    unsigned int intake_Temp = random(0, 255);
    int maf_Air_Flow_Rate = random(0, 255);

    // Work out Air Condition Measurements
    int OBTemp = picoTemp;

    int timing_Advance = 10;

    // Work out eng RPM
    float rpm_Val = rpm * 4;
    unsigned int rpm_A = (long)rpm_Val / 256;
    unsigned int rpm_B = (long)rpm_Val % 256;

    // Work out MAF values
    float maf_Val = maf_Air_Flow_Rate * 100;
    unsigned int maf_A = (long)maf_Air_Flow_Rate / 256;
    unsigned int maf_B = (long)maf_Air_Flow_Rate;

    // Build sensor return msg
    byte engine_Coolant_Temperature_Msg[8] = {3, 65, 0x05, (byte)(engine_Coolant_Temperature + 40)};
    byte engine_Rpm_Msg[8] = {4, 65, 0x0C, (byte)rpm_A, (byte)rpm_B};
    byte vehicle_Speed_Msg[8] = {3, 65, 0x0D, (byte)(vehicle_Speed)};
    byte timing_Advance_Msg[8] = {3, 65, 0x0E, (byte)((timing_Advance + 64) * 2)};
    byte intake_Temp_Msg[8] = {3, 65, 0x0F, (byte)(intake_Temp + 40)};
    byte maf_Air_Flow_Rate_Msg[8] = {4, 65, 0x10, (byte)maf_A, (byte)maf_B};
    byte Normal_DAQ[8] = {2, 0,0,0,0};

    // CAN Area
    CAN.sendMsgBuf(2, 0, 8, Normal_DAQ);
    Serial.println("Normal Data Tx: " + String((char *)Normal_DAQ));
    delay(150);

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        {

            CAN.readMsgBufID(&canId, &len, buf);
            // https://en.wikipedia.org/wiki/OBD-II_PIDs#CAN_(11-bit)_bus_format

            Serial.print("Received: ");
            Serial.print(canId, HEX);
            Serial.print(",");

            for (int i = 0; i < 3; i++)
            {
                canMessageRead = canMessageRead + buf[i] + ",";
            }
            Serial.println(canMessageRead);
            if (canMessageRead == "0,1,1,")
            {
                digitalWrite(DynoInt, HIGH);
                delay(100);
                digitalWrite(MOSUP_PIN, HIGH);
                delay(200);
                digitalWrite(DynoInt, LOW);
                digitalWrite(MOSUP_PIN, LOW);
                delay(100);
            }
            if (canMessageRead == "0,1,2,")
            {
                digitalWrite(DynoInt, HIGH);
                delay(100);
                digitalWrite(MOSDOWN_PIN, HIGH);
                delay(200);
                digitalWrite(DynoInt, LOW);
                digitalWrite(MOSDOWN_PIN, LOW);
                delay(100);
            }
            if (canMessageRead == "0,2,1,")
            {
                digitalWrite(DRS_PIN, HIGH);
                delay(50);
            }
            if (canMessageRead == "0,2,2,")
            {
                digitalWrite(DRS_PIN, LOW);
                delay(100);
            }
            //=================================================================
            // Return CAN-BUS Messages - SUPPORTED PIDs
            //=================================================================

            if (canMessageRead == "2,1,0,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x00PID);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)mode1Supported0x00PID));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            if (canMessageRead == "2,1,32,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x20PID);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)mode1Supported0x20PID));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            if (canMessageRead == "2,1,64,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, mode1Supported0x40PID);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)mode1Supported0x40PID));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            if (canMessageRead == "2,9,0,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, mode9Supported0x00PID);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)mode9Supported0x00PID));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            //=================================================================
            // Return CAN-BUS Messages - RETURN PID VALUES - SENSORS
            //=================================================================

            // Engine Coolant
            if (canMessageRead == "2,1,5,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, engine_Coolant_Temperature_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)engine_Coolant_Temperature_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // Rpm
            if (canMessageRead == "2,1,12,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, engine_Rpm_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)engine_Rpm_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // Speed
            if (canMessageRead == "2,1,13,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, vehicle_Speed_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)vehicle_Speed_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // Timing Adv
            if (canMessageRead == "2,1,14,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, timing_Advance_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)timing_Advance_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // Intake Tempture
            if (canMessageRead == "2,1,15,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, intake_Temp_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)intake_Temp_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // MAF
            if (canMessageRead == "2,1,16,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, maf_Air_Flow_Rate_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)maf_Air_Flow_Rate_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // OBD standard
            if (canMessageRead == "2,1,28,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, obd_Std_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)obd_Std_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }

            // Fuel Type Coding
            if (canMessageRead == "2,1,58,")
            {
                CAN.sendMsgBuf(REPLY_ID, 0, 8, fuel_Type_Msg);
                reply = String(String(REPLY_ID, HEX) + ",0,8," + String((char *)fuel_Type_Msg));
                Serial.println("Reply: " + reply);
                reply = "";
            }
            // VIN
            if (canMessageRead == "2,9,2,")
            {

                unsigned char frame1[8] = {16, 20, 73, 2, 1, vehicle_Vin[0], vehicle_Vin[1], vehicle_Vin[2]};
                unsigned char frame2[8] = {33, vehicle_Vin[3], vehicle_Vin[4], vehicle_Vin[5], vehicle_Vin[6], vehicle_Vin[7], vehicle_Vin[8], vehicle_Vin[9]};
                unsigned char frame3[8] = {34, vehicle_Vin[10], vehicle_Vin[11], vehicle_Vin[12], vehicle_Vin[13], vehicle_Vin[14], vehicle_Vin[15], vehicle_Vin[16]};

                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame1);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame2);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame3);
            }

            // CAL ID
            if (canMessageRead == "2,9,4,")
            {
                unsigned char frame1[8] = {16, 20, 73, 4, 1, calibration_ID[0], calibration_ID[1], calibration_ID[2]};
                unsigned char frame2[8] = {33, calibration_ID[3], calibration_ID[4], calibration_ID[5], calibration_ID[6], calibration_ID[7], calibration_ID[8], calibration_ID[9]};
                unsigned char frame3[8] = {34, calibration_ID[10], calibration_ID[11], calibration_ID[12], calibration_ID[13], calibration_ID[14], calibration_ID[15], calibration_ID[16]};

                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame1);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame2);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame3);
            }

            // ECU NAME
            if (canMessageRead == "2,9,10,")
            {

                unsigned char frame1[8] = {10, 14, 49, 10, 01, ecu_Name[0], ecu_Name[1], ecu_Name[2]};
                unsigned char frame2[8] = {21, ecu_Name[3], ecu_Name[4], ecu_Name[5], ecu_Name[6], ecu_Name[7], ecu_Name[8], ecu_Name[9]};
                unsigned char frame3[8] = {22, ecu_Name[10], ecu_Name[11], ecu_Name[12], ecu_Name[13], ecu_Name[14], ecu_Name[15], ecu_Name[16]};
                unsigned char frame4[8] = {23, ecu_Name[17], ecu_Name[18]};

                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame1);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame2);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame3);
                CAN.sendMsgBuf(REPLY_ID, 0, 8, frame4);
            }
            // DTC
            if (canMessageRead == "1,3,0,")
            {
                if (MIL)
                {
                    unsigned char DTC[] = {6, 67, 1, 2, 23, 0, 0, 0}; // P0217
                    CAN.sendMsgBuf(REPLY_ID, 0, 8, DTC);
                    reply = String(String(REPLY_ID, HEX) + ",0,8, " + String((char *)DTC));
                    Serial.println("Reply: " + reply);
                    reply = "";
                }
                else
                {
                    unsigned char DTC[] = {6, 67, 0, 0, 0, 0, 0, 0}; // No Stored DTC
                    CAN.sendMsgBuf(REPLY_ID, 0, 8, DTC);
                    reply = String(String(REPLY_ID, HEX) + ",0,8, " + String((char *)DTC));
                    Serial.println("Reply: " + reply);
                    reply = "";
                }
            }

            // DTC Clear
            if (canMessageRead == "1,4,0,")
            {
                MIL = false;
            }

            canMessageRead = "";
        }
    }

    delay(100);
}

// ——————————————————————————————————————————————————————————————————————————————
// Extra Functions Configuration
// ——————————————————————————————————————————————————————————————————————————————

void MCP2515_Init(void)
{
    while (!CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        blink(100);
        blink(100);
        Serial.println("Error: Check MCP2515 Module Wiring.");
        Serial.println("Retrying in 3 Seconds.");
        blink(500);
        blink(500);
        blink(500);
    }
    Serial.println("CAN Status: OK");
}

void blink(int deltime)
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(deltime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(deltime);
}

#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif
