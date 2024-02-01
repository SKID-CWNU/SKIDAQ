// ——————————————————————————————————————————————————————————————————————————————//
//                                    SKIDAQ                                     //
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                Solenoid Valve Controlled QuickShift Interface                 //
//                             Author: Lim Chae Won                              //
//                             The MIT License (MIT)                             //
// ——————————————————————————————————————————————————————————————————————————————//
//
//    Pin Summary
//
//    GP4  - ADXL345 SDA Pin
//    GP5  - ADXL345 SCL Pin
//    GP7  - DIAG Mode Interrupt
//    GP10 - DHT Temp/Humid Sensor
//    GP12 - MOSFET Upshift
//    GP13 - MOSFET Downshift
//    GP16 - CAN RX Pin - to SO (MISO)
//    GP17 - CAN CS Pin
//    GP18 - CAN SCK Pin
//    GP19 - CAN TX Pin - to SI (MOSI)
//    GP20 - CAN Interrupt Pin
//    ADC0 - CKP(RPM) Pulse Monitor
//
//    Ref: https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg
//
// ——————————————————————————————————————————————————————————————————————————————//

#include <ADCInput.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include "DFRobot_MCP2515.h"
#include <SPI.h>
#include <Wire.h>

// ——————————————————————————————————————————————————————————————————————————————
//    DHT Temp/Humidity Sensor Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define DHTPIN 10     // DHT Signal Pin
#define DHTTYPE DHT22 // DHT Sensor type: DHT22
DHT dht(DHTPIN, DHTTYPE);

// ——————————————————————————————————————————————————————————————————————————————
//    CAN Interface Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define INT32U unsigned long int
DFRobot_MCP2515 CAN(17); // Set CS to pin 17
const int CAN_INT_PIN = 20;
INT32U canId = 0x000;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
String BuildMessage = "";
int MSGIdentifier = 0;

// ——————————————————————————————————————————————————————————————————————————————
//    ADXL345 Acceleration Sensor Configuration
// ——————————————————————————————————————————————————————————————————————————————
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
    sensor_t sensor;
    accel.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" m/s^2");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" m/s^2");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" m/s^2");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displayDataRate(void)
{
    Serial.print("Data Rate:    ");

    switch (accel.getDataRate())
    {
    case ADXL345_DATARATE_3200_HZ:
        Serial.print("3200 ");
        break;
    case ADXL345_DATARATE_1600_HZ:
        Serial.print("1600 ");
        break;
    case ADXL345_DATARATE_800_HZ:
        Serial.print("800 ");
        break;
    case ADXL345_DATARATE_400_HZ:
        Serial.print("400 ");
        break;
    case ADXL345_DATARATE_200_HZ:
        Serial.print("200 ");
        break;
    case ADXL345_DATARATE_100_HZ:
        Serial.print("100 ");
        break;
    case ADXL345_DATARATE_50_HZ:
        Serial.print("50 ");
        break;
    case ADXL345_DATARATE_25_HZ:
        Serial.print("25 ");
        break;
    case ADXL345_DATARATE_12_5_HZ:
        Serial.print("12.5 ");
        break;
    case ADXL345_DATARATE_6_25HZ:
        Serial.print("6.25 ");
        break;
    case ADXL345_DATARATE_3_13_HZ:
        Serial.print("3.13 ");
        break;
    case ADXL345_DATARATE_1_56_HZ:
        Serial.print("1.56 ");
        break;
    case ADXL345_DATARATE_0_78_HZ:
        Serial.print("0.78 ");
        break;
    case ADXL345_DATARATE_0_39_HZ:
        Serial.print("0.39 ");
        break;
    case ADXL345_DATARATE_0_20_HZ:
        Serial.print("0.20 ");
        break;
    case ADXL345_DATARATE_0_10_HZ:
        Serial.print("0.10 ");
        break;
    default:
        Serial.print("???? ");
        break;
    }
    Serial.println(" Hz");
}

void displayRange(void)
{
    Serial.print("Range:         +/- ");

    switch (accel.getRange())
    {
    case ADXL345_RANGE_16_G:
        Serial.print("16 ");
        break;
    case ADXL345_RANGE_8_G:
        Serial.print("8 ");
        break;
    case ADXL345_RANGE_4_G:
        Serial.print("4 ");
        break;
    case ADXL345_RANGE_2_G:
        Serial.print("2 ");
        break;
    default:
        Serial.print("?? ");
        break;
    }
    Serial.println(" g");
}

// ——————————————————————————————————————————————————————————————————————————————
//    Other Comp. Initalization
// ——————————————————————————————————————————————————————————————————————————————
int picoTemp = analogReadTemp(); // Pi Pico On-Board Temp Sensor
int obled = LED_BUILTIN;         // On-Board LED for Basic Error Indication
ADCInput RPulse(A0);             // For stereo/dual mikes, could use this line instead
// ADCInput(A0, A1);
#define DiagEN 7 // Toggle Switch for OBD2 Simulation
int DIAGENB = 0; //

// ——————————————————————————————————————————————————————————————————————————————
//    MOSFET Switch Module Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define MOSUP_PIN 12
#define MOSDOWN_PIN 13

// ——————————————————————————————————————————————————————————————————————————————
//    Main Program Setup
// ——————————————————————————————————————————————————————————————————————————————
void setup()
{
    delay(2000);
    pinMode(obled, OUTPUT);
    pinMode(MOSUP_PIN, OUTPUT);
    pinMode(MOSDOWN_PIN, OUTPUT);
    pinMode(DiagEN, INPUT);
    Serial.begin(115200);
    dht.begin();        // DHT Sensor Begin
    RPulse.begin(8000); // CPS Pulse A/D Converter Begin
    if (!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1)
            ;
    }
    //accel.setRange(ADXL345_RANGE_16_G);
    // accel.setRange(ADXL345_RANGE_8_G);
    // accel.setRange(ADXL345_RANGE_4_G);
    accel.setRange(ADXL345_RANGE_2_G);
    Serial.println("-----ADXL345 STATUS-----");
    /* Display some basic information on this sensor */
    displaySensorDetails();
    /* Display additional settings (outside the scope of sensor_t) */
    displayDataRate();
    displayRange();
    Serial.println("");

    while (CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Please Init CAN BUS Shield again");
        digitalWrite(obled, HIGH);
        delay(3000);
        digitalWrite(obled, LOW);
    }
    delay(3000);
}
// ——————————————————————————————————————————————————————————————————————————————
//    Main(Loop) Area
// ——————————————————————————————————————————————————————————————————————————————
void loop()
{
    // DHT Area
    // ——————————————————————————————————————————————————————————————————————————————
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

    // ——————————————————————————————————————————————————————————————————————————————
    //    ADXL345 Acceleration Sensor
    // ——————————————————————————————————————————————————————————————————————————————
    sensors_event_t event;
    accel.getEvent(&event);

    // Display the results (acceleration is measured in m/s^2)
    // Serial.print("X: ");
    // Serial.print(event.acceleration.x);
    // Serial.print("  ");
    // Serial.print("Y: ");
    // Serial.print(event.acceleration.y);
    // Serial.print("  ");
    // Serial.print("Z: ");
    // Serial.print(event.acceleration.z);
    // Serial.print("  ");
    // Serial.println("m/s^2 ");

    // CAN Area

    // Sensor Value conversion
    char rndCoolantTemp = random(1, 200); // Set to random until measuring Actual value
    char rndRPM = random(1, 55);
    char rndSpeed = random(0, 255);
    char rndIAT = random(0, 255);
    char rndMAF = random(0, 255);
    char acelx = event.acceleration.x;
    char acely = event.acceleration.y;
    char acelz = event.acceleration.z;
    char ControlAirTemp = t;
    char OBTemp = picoTemp;
    char ControlHumidity = h;

    // Normal CAN Message Configuration
    unsigned char normal1MSG[8] = {rndCoolantTemp, rndRPM, rndSpeed, rndIAT, rndMAF, ControlAirTemp, ControlHumidity, 0};
    unsigned char normal2MSG[4] = {acelx, acely, acelz, 1};
    CAN.sendMsgBuf(0x0, 0, 8, normal1MSG);
    delay(200);
    CAN.sendMsgBuf(0x1, 0, 4, normal2MSG);
    delay(200);

    // GENERAL Sensor ROUTINE
    unsigned char SupportedPID[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    unsigned char MilCleared[7] = {4, 65, 63, 34, 224, 185, 147};

    // OBD2 Mode Configuration
    // SENSOR CAN Message Setup
    unsigned char dCoolantTemp[7] = {4, 65, 5, rndCoolantTemp, 0, 185, 147};
    unsigned char drpm[7] = {4, 65, 12, rndRPM, 224, 185, 147};
    unsigned char dvspeed[7] = {4, 65, 13, rndSpeed, 224, 185, 147};
    unsigned char dIATSensor[7] = {4, 65, 15, rndIAT, 0, 185, 147};
    unsigned char dMAFSensor[7] = {4, 65, 16, rndMAF, 0, 185, 147};
    unsigned char dBoardAirTemp[7] = {4, 65, 70, ControlAirTemp, 0, 185, 147};
    unsigned char dControlHum[7] = {4, 65, 71, ControlHumidity, 0, 185, 147};
    unsigned char dAcelxdir[7] = {4, 65, 77, acelx, 224, 185, 147};
    unsigned char dAcelydir[7] = {4, 65, 78, acely, 224, 185, 147};
    unsigned char dAcelzdir[7] = {4, 65, 79, acelz, 224, 185, 147};
    unsigned char dAcelall[7] = {4, 65, 80, acelx, acely, acelz, 147};

    DIAGENB = digitalRead(DiagEN); // When Diag Pin toggled to on

    if (DIAGENB == 1)
    {
        Serial.println("OBD-2 Diag Mode Enabled");
    }
    while (DIAGENB == 1)
    {

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
                Serial.println("OBD-2 Diag Mode Cleared");
                delay(100);
                BuildMessage = "";
                break;
            }

            // SEND SENSOR STATUSES
            if (BuildMessage == "2,1,5,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dCoolantTemp);
            }
            if (BuildMessage == "2,1,12,0,0,0,0,0,")
            {
                Serial.println(rndRPM);
                CAN.sendMsgBuf(0x7E8, 0, 7, drpm);
            }
            if (BuildMessage == "2,1,13,0,0,0,0,0,")
            {
                Serial.println(rndSpeed);
                CAN.sendMsgBuf(0x7E8, 0, 7, dvspeed);
            }
            if (BuildMessage == "2,1,15,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dIATSensor);
            }
            if (BuildMessage == "2,1,16,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dMAFSensor);
            }
            if (BuildMessage == "2,1,70,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dBoardAirTemp);
                Serial.println(t);
            }
            if (BuildMessage == "2,1,71,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dControlHum);
            }
            if (BuildMessage == "2,1,77,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dAcelxdir);
            }
            if (BuildMessage == "2,1,78,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dAcelydir);
            }
            if (BuildMessage == "2,1,79,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dAcelzdir);
            }
            if (BuildMessage == "2,1,80,0,0,0,0,0,")
            {
                CAN.sendMsgBuf(0x7E8, 0, 7, dAcelzdir);
            }
            BuildMessage = "";
        }
        delay(100);
    }
    DIAGENB = 0;
    delay(100);
}

// ——————————————————————————————————————————————————————————————————————————————
//   OBD-II PIDs Configuration - https://en.wikipedia.org/wiki/OBD-II_PIDs
// ——————————————————————————————————————————————————————————————————————————————

// Extra Configurations
#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif
