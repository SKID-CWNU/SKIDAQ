// Library
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>
#include "DFRobot_MCP2515.h"
#include <RtcDS3231.h>
#include <SPI.h>
#include <Wire.h>

// Comps
// Rpi Pico, CAN Transceiver, ADC Module, 9 axis module, GPS(NOT WORKING)
// MOSFET Switch(2, up, down), HDS Interpreter, RTC CLOCK, Telemetry(not used)

DFRobot_MCP2515 CAN(17); // Set CS to pin 17
const int CAN_INT_PIN = 20;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_ADS1115 ads; /* ADS1115 - 16-bit version */

RtcDS3231<TwoWire> Rtc(Wire);

#define DynoPin 9
#define MOSUP_PIN 12
#define MOSDOWN_PIN 13

char picoTemp = analogReadTemp(); // Pi Pico On-Board Temp Sensor
int obled = LED_BUILTIN;          // On-Board LED for Basic Error Indication

void ADS1115_Init(void);
void ADS1115_Read(void);
void ADXL345_Init(void);
void ADXL345_displaySensorDetails(void);
void ADXL345_displayDataRate(void);
void ADXL345_displayRange(void);
void MCP2515_Init(void);
void upShiftFunc(void);
void downShiftFunc(void);
void blink(int deltime);



void setup()
{
    delay(2000);
    pinMode(obled, OUTPUT);
    pinMode(MOSUP_PIN, OUTPUT);
    pinMode(MOSDOWN_PIN, OUTPUT);
    pinMode(DynoPin, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    ADS1115_Init();
    ADXL345_Init();
    MCP2515_Init();
    digitalWrite(DynoPin, LOW);
    blink(1000);
    delay(1000);
}

// ——————————————————————————————————————————————————————————————————————————————
//    Main(Loop) Area
// ——————————————————————————————————————————————————————————————————————————————
void loop()
{
    ADS1115_Read();
    sensors_event_t event;
    accel.getEvent(&event);
    // Work out ADXL345 Output to CAN
    int acelx = event.acceleration.x;
    int acely = event.acceleration.y;
    int acelz = event.acceleration.z;
    int OBTemp = picoTemp;
}

void ADS1115_Init(void)
{
    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    //                                                                ADS1015  ADS1115
    //                                                                -------  -------
    // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/-
    if (!ads.begin(0x49))
    {
        Serial.println("Error: Check ADS1115 Wiring");
        while (1)
            ;
    }
    // Setup 3V comparator on channel 0
    ads.startComparator_SingleEnded(0, 1000);
}

void ADS1115_Read(void)
{
    int16_t adc0;

    // Comparator will only de-assert after a read
    adc0 = ads.getLastConversionResults();
}

void ADXL345_Init(void)
{
    while (!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        blink(100);
        Serial.println("Error: Check ADXL345 Wiring.");
        Serial.println("Retrying in 3 Seconds.");
        blink(500);
        blink(500);
    }
    Serial.println("ADXL345 Status: OK");
    // /* Display some basic information on this sensor */
    // displaySensorDetails();
    // /* Display additional settings (outside the scope of sensor_t) */
    // displayDataRate();
    // displayRange();
    Serial.println("");
    // accel.setRange(ADXL345_RANGE_16_G);
    //  accel.setRange(ADXL345_RANGE_8_G);
    //  accel.setRange(ADXL345_RANGE_4_G);
    accel.setRange(ADXL345_RANGE_2_G);
}

void ADXL345_displayDataRate(void)
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

void ADXL345_displayRange(void)
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

void ADXL345_displaySensorDetails(void)
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

void upShiftFunc(int time) {
    digitalWrite(MOSUP_PIN, HIGH);
    digitalWrite(DynoPin, HIGH);
    delay(time);
    digitalWrite(MOSUP_PIN, LOW);
    digitalWrite(DynoPin, LOW);
}

void downShiftFunc(int time) {
    digitalWrite(MOSDOWN_PIN, HIGH);
    digitalWrite(DynoPin, HIGH);
    delay(time);
    digitalWrite(MOSDOWN_PIN, LOW);
    digitalWrite(DynoPin, LOW);
}

void blink(int deltime)
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(deltime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(deltime);
}
