/* Pin Summary

    GP12 - MOSFET Upshift
    GP13 - MOSFET Downshift
    GP14 - DRS Switch
    GP16 - CAN RX Pin - to SO (MISO)
    GP17 - CAN CS Pin
    GP18 - CAN SCK Pin
    GP19 - CAN TX Pin - to SI (MOSI)
    GP20 - CAN Interrupt Pin

    Ref: https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg

 —————————————————————————————————————————————————————————————————————————————— */

#include "DFRobot_MCP2515.h"

#define upSW 12
#define downSW 13
#define drsSW 14

DFRobot_MCP2515 CAN(17); // Set CS to pin 17
const int CAN_INT_PIN = 20;
// This is the length of the incoming CAN-BUS message
unsigned char len = 0;
// Default reply ECU ID
#define REPLY_ID 0x7E8
long unsigned int canId = 0x000;
// This the eight byte buffer of the incoming message data payload
unsigned char buf[8];
String BuildMessage = "";
unsigned char flagRecv = 0;

void blink(int deltime)
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(deltime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(deltime);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(upSW, INPUT_PULLUP);
    pinMode(downSW, INPUT_PULLUP);
    pinMode(drsSW, INPUT_PULLUP);
    while (CAN.begin(CAN_500KBPS))
    { // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Please Init CAN BUS Shield again");
        blink(500);
        blink(500);
    }
    delay(1000);
}
static char buff[100];
unsigned char up[8] = {0, 1, 1, 0, 0, 0, 0, 0};
unsigned char down[8] = {0, 1, 2, 0, 0, 0, 0, 0};
unsigned char drstrigon[8] = {0, 2, 1, 0, 0, 0, 0, 0};
unsigned char drstrigoff[8] = {0, 2, 2, 0, 0, 0, 0, 0};
unsigned char pidchk[8] = {2, 1, 0, 0, 0, 0, 0, 0};
unsigned char milClr[8] = {2, 1, 1, 0, 0, 0, 0, 0};
unsigned char coolTemp[8] = {2, 1, 5, 0, 0, 0, 0, 0};
unsigned char rpmm[8] = {2, 1, 12, 0, 0, 0, 0, 0};
unsigned char ambtemp[8] = {2, 1, 70, 0, 0, 0, 0, 0};

void shiftMode()
{
    int upval = digitalRead(upSW);
    int downval = digitalRead(downSW);
    if (upval == 1 && downval == 0)
    {
        CAN.sendMsgBuf(0x00, 0, 8, up);
        delay(400);
    }
    else if (upval == 0 && downval == 1)
    {
        CAN.sendMsgBuf(0x00, 0, 8, down);
        delay(400);
    }
    else if (upval == downval)
    {
        delay(60);
    }
}

void drsMode()
{
    int drsval = digitalRead(drsSW);
    if (drsval == 1)
    {
        CAN.sendMsgBuf(0x00, 0, 8, drstrigon);
        delay(50);
    } else if (drsval == 0) {
         CAN.sendMsgBuf(0x00, 0, 8, drstrigoff);
        delay(50);
    }
}

void loop()
{

    shiftMode();
    drsMode();
    delay(50);
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
    static int p;
    char b[90];

    if (CAN_MSGAVAIL == CAN.checkReceive())
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
        }
        else if (canId == 2)
            // print the data
            for (int i = 0; i < len; i++)
            {
                BuildMessage = BuildMessage + buf[i] + ",";
            }
        Serial.println("DAQ Rx: ");
        Serial.print(BuildMessage);
        BuildMessage = "";
        Serial.println(" ");
    }
    delay(100);
}
