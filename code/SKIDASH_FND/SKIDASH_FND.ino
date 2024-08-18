
/*
 *  74HC595 Seven Segment 2 Digits LED Display with decimal points for Arduino
 *  updated by Ahmad Shamshiri for Robojax
 *  On Monday Sep 17, 2019 at 00:03 in Ajax, Ontario, Canada
 *
 *  At the moments, it doesn't display decimal points. It needs a little work to make it work
 *  Original source and text.
 *  2 Digitl 7 segment display PCB board with (2) 74HC595 shift register ICs
 *  Arduino Tutorial - www.Ardumotive.com
 *  Dev: Michalis Vasilakis // Date: 31/1/2018 // Ver:1.0
 */
#include <ShiftRegister74HC595.h>
#include "DFRobot_MCP2515.h"
#define SDI 7
#define SCLK 6
#define LOAD 5
#define DIGITS 2
#define SDI 7
#define SCLK 6
#define LOAD 5
#define DIGITS 2

const int SPI_CS_PIN = 17;
DFRobot_MCP2515 CAN(SPI_CS_PIN); // Set CS pin

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
String BuildMessage = "";
// create shift register object (number of shift registers, data pin, clock pin, latch pin)
ShiftRegister74HC595 sr(DIGITS, SDI, SCLK, LOAD);
ShiftRegister74HC595 srw();

int value, digit1, digit2, digit3, digit4;
uint8_t digits[] = {
    B11000000, // 0
    B11111001, // 1
    B10100100, // 2
    B10110000, // 3
    B10011001, // 4
    B10010010, // 5
    B10000010, // 6
    B11111000, // 7
    B10000000, // 8
    B10010000  // 9
};

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

void loop()
{
    showNumber(18);
    delay(2000); //
    for (int i = 0; i <= 99; i++)
    {
        showNumber(i);
        delay(100);
    }
    delay(2000);
    for (int i = 99; i >= 0; i--)
    {
        showNumber(i);
        delay(50);
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
    delay(2000);
} // loop

/*
 * @brief shows number on the Seven Segment Display
 * @param "num" is integer
 * @return returns none
 * Usage to show 18: showNumber(18);
 * Written by Ahmad Shamshiri on Sep 17, 2019.
 * in Ajax, Ontario, Canada
 * www.Robojax.com
 */
void showNumber(int num)
{
    digit2 = num % 10;
    digit1 = (num / 10) % 10;
    // Send them to 7 segment displays
    uint8_t numberToPrint[] = {digits[digit2], digits[digit1]};
    sr.setAll(numberToPrint);
}