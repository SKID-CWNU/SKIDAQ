// ——————————————————————————————————————————————————————————————————————————————//
//                                    SKIDAQ                                     //
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                Solenoid Valve Controlled QuickShift Interface                 //
//                             Author: Lim Chae Won                              //
//                             The MIT License (MIT)                             //
// ——————————————————————————————————————————————————————————————————————————————//
/*!
 * @file  DIAG_DAQ.ino
 * @brief  Raspberry Pi Pico based SKID-FS DAQ System Diganosing Device
 * @n  the data coming can't be too fast, must >20ms, or else you can use check mode
 * @copyright  Copyright (c) Lim Chae Won 2024
 * @license  The MIT License (MIT)
 * @author  SKID-CWNU (Lim Chae Won)
 * @maintainer Lim Chae Won
 * @version  V1.0
 * @date  2024-01-20
 * @url
 * */
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
    delay(3000);
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
unsigned char obhumidity[8] = {2, 1, 71, 0, 0, 0, 0, 0};
unsigned char Axdir[8] = {2, 1, 77, 0, 0, 0, 0, 0};
unsigned char Aydir[8] = {2, 1, 78, 0, 0, 0, 0, 0};
unsigned char Azdir[8] = {2, 1, 79, 0, 0, 0, 0, 0};
unsigned char AAll[8] = {2, 1, 80, 0, 0, 0, 0, 0};

void loop()
{

    long Order = Serial.read();
    if (Order == 'c')
    {
        CAN.sendMsgBuf(0x02, 0, 8, pidchk);
        Order = '0';
    }
    else if (Order == '0')
    {
        CAN.sendMsgBuf(0x02, 0, 8, milClr);
        Order = '0';
    }
    else if (Order == 'l')
    {
        CAN.sendMsgBuf(0x02, 0, 8, coolTemp);
        Order = '0';
    }
    else if (Order == 'r')
    {
        CAN.sendMsgBuf(0x02, 0, 8, rpmm);
        Order = '0';
    }
    else if (Order == 't')
    {
        CAN.sendMsgBuf(0x02, 0, 8, ambtemp);
        Order = '0';
    }
    else if (Order == 'h')
    {
        CAN.sendMsgBuf(0x02, 0, 8, obhumidity);
        Order = '0';
    }
    else if (Order == 'x')
    {
        CAN.sendMsgBuf(0x02, 0, 8, Axdir);
        Order = '0';
    }
    else if (Order == 'y')
    {
        CAN.sendMsgBuf(0x02, 0, 8, Aydir);
        Order = '0';
    }
    else if (Order == 'z')
    {
        CAN.sendMsgBuf(0x02, 0, 8, Azdir);
        Order = '0';
    }
    else if (Order == 'a')
    {
        CAN.sendMsgBuf(0x02, 0, 8, AAll);
        Order = '0';
    }

    else if (Order == '0')
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
            if (canId == 0x1)
            {

            }

            if (canId == 0x1)
            {
                
            }
            if (canId == 0x7E8)
            {
                if (buf[0] == 4 && buf[1] == 65)
                {
                    switch (buf[2])
                    {
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
                    case 13:
                        Serial.print("Speed: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 15:
                        Serial.print("IAT Temp: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 70:
                        Serial.print("Ambient Temperature: ");
                        Serial.println(buf[3]);
                        break;
                    case 71:
                        Serial.print("Humidity: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 77:
                        Serial.print("Accel X: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 78:
                        Serial.print("Accel Y: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 79:
                        Serial.print("Accel Z: ");
                        Serial.println(buf[3], OCT);
                        break;
                    case 80:
                        Serial.print("Accel All Axis: ");
                        Serial.println(buf[3], OCT);
                        Serial.println(buf[4], OCT);
                        Serial.println(buf[5], OCT);
                        break;

                    default:
                        break;
                    }
                }
            }
            else
            {
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
}
