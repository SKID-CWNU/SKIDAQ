// ——————————————————————————————————————————————————————————————————————————————//
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                Solenoid Valve Controlled QuickShift Interface                 //
//                             Author: Lim Chae Won                              //
// ——————————————————————————————————————————————————————————————————————————————//

#include "DHT.h"
#include <SoftwareSerial.h>
#include "DFRobot_MCP2515.h"

// ——————————————————————————————————————————————————————————————————————————————
//    DHT Library Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define DHTPIN 10     // DHT Signal Pin
#define DHTTYPE DHT22 // DHT Sensor type: DHT22
DHT dht(DHTPIN, DHTTYPE);

// ——————————————————————————————————————————————————————————————————————————————
//    CAN Interface Configuration
// ——————————————————————————————————————————————————————————————————————————————
const int SPI_CS_PIN = 17;
DFRobot_MCP2515 CAN(SPI_CS_PIN); // Set CS pin
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
// ——————————————————————————————————————————————————————————————————————————————
//   OBD-II PIDs Configuration - https://en.wikipedia.org/wiki/OBD-II_PIDs
// ——————————————————————————————————————————————————————————————————————————————
#define PID_ENGIN_PRM 0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_COOLANT_TEMP 0x05

#define CAN_ID_PID 0x7DF

unsigned char PID_INPUT;
unsigned char getPid = 0;

void set_mask_filt()
{
    /*
        set mask, set both the mask to 0x3ff
    */

    CAN.initMask(0, 0, 0x7FC);
    CAN.initMask(1, 0, 0x7FC);

    /*
        set filter, we can receive id from 0x04 ~ 0x09
    */
    CAN.initFilter(MCP2515_RXF0, 0, 0x7E8);
    CAN.initFilter(MCP2515_RXF1, 0, 0x7E8);

    CAN.initFilter(MCP2515_RXF2, 0, 0x7E8);
    CAN.initFilter(MCP2515_RXF3, 0, 0x7E8);
    CAN.initFilter(MCP2515_RXF4, 0, 0x7E8);
    CAN.initFilter(MCP2515_RXF5, 0, 0x7E8);
}

// ——————————————————————————————————————————————————————————————————————————————
//    Main Program Setup
// ——————————————————————————————————————————————————————————————————————————————
void setup()
{
    //--- Switch on builtin led
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    //--- Start serial
    Serial.begin(115200);
    //--- Wait for serial (blink led at 10 Hz during waiting)
    while (!Serial)
    {
        delay(50);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    while (CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("DFROBOT's CAN BUS Shield init fail");
        Serial.println("Please Init CAN BUS Shield again");
        delay(3000);
    }
    Serial.println("DFROBOT's CAN BUS Shield init ok!\n");

    attachInterrupt(11, MCP2515_ISR, FALLING); // start interrupt
    
    Serial.println("DAQ System Init Success!");
    set_mask_filt();
    delay(5000);
}
int h = 0;
int t = 0;

// ——————————————————————————————————————————————————————————————————————————————
//    Main(Loop) Area
// ——————————————————————————————————————————————————————————————————————————————
void loop()
{
    // DHT Area
    h = dht.readHumidity();    // Gets Humidity value
    t = dht.readTemperature(); // Gets Temperature value

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    // Printing Values for Monitoring
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print("%  Temperature: ");
    Serial.print(t);
    // ——————————————————————————————————————————————————————————————————————————————

    // CAN Area

    // set id to send data buff, id is arranged form 0x00 to 0x09.
    CAN.sendMsgBuf(2, 0, sizeof(h), h);
    delay(100);
}

// Extra Configurations
#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif
