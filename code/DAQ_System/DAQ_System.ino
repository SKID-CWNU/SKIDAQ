// ——————————————————————————————————————————————————————————————————————————————//
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                Solenoid Valve Controlled QuickShift Interface                 //
//                             Author: Lim Chae Won                              //
// ——————————————————————————————————————————————————————————————————————————————//

#include "DHT.h"
#include <SoftwareSerial.h>
#include <ACAN2515.h>

// ——————————————————————————————————————————————————————————————————————————————
//    DHT Library Configuration
// ——————————————————————————————————————————————————————————————————————————————
#define DHTPIN 10     // DHT Signal Pin
#define DHTTYPE DHT22 // DHT Sensor type: DHT22
DHT dht(DHTPIN, DHTTYPE);

// ——————————————————————————————————————————————————————————————————————————————
//    CAN Interface Configuration
// ——————————————————————————————————————————————————————————————————————————————
static const byte MCP2515_SCK = 18;  // SCK input of MCP2515
static const byte MCP2515_MOSI = 19; // SDI input of MCP2515
static const byte MCP2515_MISO = 16; // SDO output of MCP2515
static const byte MCP2515_CS = 17;   // CS input of MCP2515
static const byte MCP2515_INT = 20;  // INT output of MCP2515
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);
static const uint32_t QUARTZ_FREQUENCY = 20UL * 1000UL * 1000UL; // 20 MHz

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
    //--- SPI Pin Assignment
    SPI.setSCK(MCP2515_SCK);
    SPI.setTX(MCP2515_MOSI);
    SPI.setRX(MCP2515_MISO);
    SPI.setCS(MCP2515_CS);
    SPI.begin();                                                 //--- Begin SPI
    dht.begin();                                                 //--- Begin DHT
    ACAN2515Settings settings(QUARTZ_FREQUENCY, 125UL * 1000UL); // CAN bit rate 125 kb/s
    settings.mRequestedMode = ACAN2515Settings::NormalMode;      // Select Normal mode
    const uint16_t errorCode = can.begin(settings, []
                                         { can.isr(); });
    if (errorCode == 0)
    {
        Serial.print("Bit Rate prescaler: ");
        Serial.println(settings.mBitRatePrescaler);
        Serial.print("Propagation Segment: ");
        Serial.println(settings.mPropagationSegment);
        Serial.print("Phase segment 1: ");
        Serial.println(settings.mPhaseSegment1);
        Serial.print("Phase segment 2: ");
        Serial.println(settings.mPhaseSegment2);
        Serial.print("SJW: ");
        Serial.println(settings.mSJW);
        Serial.print("Triple Sampling: ");
        Serial.println(settings.mTripleSampling ? "yes" : "no");
        Serial.print("Actual bit rate: ");
        Serial.print(settings.actualBitRate());
        Serial.println(" bit/s");
        Serial.print("Exact bit rate ? ");
        Serial.println(settings.exactBitRate() ? "yes" : "no");
        Serial.print("Sample point: ");
        Serial.print(settings.samplePointFromBitStart());
        Serial.println("%");
    }
    else
    {
        Serial.print("Configuration error 0x");
        Serial.println(errorCode, HEX);
    }
    Serial.println("DAQ System Init Success!");
    delay(5000);
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
    CANMessage frame;
    if (gBlinkLedDate < millis())
    {
        gBlinkLedDate += 2000;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        const bool ok = can.tryToSend(frame);
        if (ok)
        {
            gSentFrameCount += 1;
            Serial.print("Sent: ");
            Serial.println(gSentFrameCount);
        }
        else
        {
            Serial.println("Send failure");
        }
    }
    if (can.available())
    {
        can.receive(frame);
        gReceivedFrameCount++;
        Serial.print("Received: ");
        Serial.println(gReceivedFrameCount);
    }
}

// Extra Configurations
static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;


#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif
