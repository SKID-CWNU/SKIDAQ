//——————————————————————————————————————————————————————————————————————————————
/*!
 * @file  receiveInterrupt.ino
 * @brief  CAN-BUS Shield, receive data with interrupt mode when in interrupt mode,
 * @n  the data coming can't be too fast, must >20ms, or else you can use check mode
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  Arduinolibrary
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2022-05-25
 * @url  https://github.com/DFRobot/DFRobot_MCP2515

 * Pin Summary
 
    GP4  - ADXL345 SDA Pin
    GP5  - ADXL345 SCL Pin
    GP6  - DIAG Mode Interrupt
    GP10 - DHT Temp/Humid Sensor
    GP12 - MOSFET Upshift
    GP13 - MOSFET Downshift
    GP16 - CAN RX Pin - to SO (MISO)
    GP17 - CAN CS Pin
    GP18 - CAN SCK Pin
    GP19 - CAN TX Pin - to SI (MOSI)
    GP20 - CAN Interrupt Pin
    ADC0 - CKP(RPM) Pulse Monitor

    Ref: https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg

 —————————————————————————————————————————————————————————————————————————————— */

#include "DFRobot_MCP2515.h"

const int SPI_CS_PIN = 17;
DFRobot_MCP2515 CAN(SPI_CS_PIN); // Set CS pin

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
String BuildMessage = "";

void setup()
{
    Serial.begin(115200);

    while (CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("DFROBOT's CAN BUS Shield init fail");
        Serial.println("Please Init CAN BUS Shield again");
        delay(3000);
    }
    Serial.println("DFROBOT's CAN BUS Shield init ok!\n");

    attachInterrupt(20, MCP2515_ISR, FALLING); // start interrupt
}

void MCP2515_ISR()
{
    flagRecv = 1;
}
#define INT32U unsigned long int
INT32U canId = 0x000;
unsigned char pidchk[8] = {2, 1, 0, 0, 0, 0, 0, 0};
unsigned char milClr[8] = {2, 1, 1, 0, 0, 0, 0, 0};
unsigned char coolTemp[8] = {2, 1, 5, 0, 0, 0, 0, 0};
unsigned char rpmm[8] = {2, 1, 12, 0, 0, 0, 0, 0};
unsigned char ambtemp[8] = {2, 1, 70, 0, 0, 0, 0, 0};
void loop()
{

    char Order = Serial.read();
    if (Order == '1')
    {
        CAN.sendMsgBuf(0x02, 0, 8, pidchk);
        Order = 0;
    }
    else if (Order == '2')
    {
        CAN.sendMsgBuf(0x02, 0, 8, milClr);
        Order = 0;
    }
    else if (Order == '3')
    {
        CAN.sendMsgBuf(0x02, 0, 8, coolTemp);
        Order = 0;
    }
    else if (Order == '4')
    {
        CAN.sendMsgBuf(0x02, 0, 8, rpmm);
        Order = 0;
    }
    else if (Order == '5')
    {
        CAN.sendMsgBuf(0x02, 0, 8, ambtemp);
        Order = 0;
    }
    else if (Order == 0)
    {
    }
    if (flagRecv)
    { // check if get data

        flagRecv = 0; // clear flag

        // iterate over all pending messages
        // If either the bus is saturated or the MCU is busy,
        // both RX buffers may be in use and after having read a single
        // message, MCU does  clear the corresponding IRQ conditon.
        while (CAN_MSGAVAIL == CAN.checkReceive())
        {
            // read data,  len: data length, buf: data buf
            CAN.readMsgBuf(&len, buf);
            canId = CAN.getCanId();
            if (canId == 0x7E8)
            {
                if (buf[0] == 4 && buf[1] == 65)
                {
                    switch (buf[2])
                    {
                    case 70:
                        Serial.print("Ambient Temperature: ");
                        Serial.println(buf[3]);
                        break;
                    case 63:
                        Serial.println("Status Cleared.");
                        break;
                    case 5:
                        Serial.print("Coolant Temperature: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 12:
                        Serial.print("TachoMeter: ");
                        Serial.println(buf[3], OCT);
                        break;

                    default:
                        break;
                    }
                }
            } else if (canId)
            // print the data
            for (int i = 0; i < len; i++)
            {
                BuildMessage = BuildMessage + buf[i] + ",";
            }
            Serial.println(BuildMessage);
            BuildMessage = "";
            Serial.println(" ");
        }
    }
}
