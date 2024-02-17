// ——————————————————————————————————————————————————————————————————————————————
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

    GP12 - MOSFET Upshift
    GP13 - MOSFET Downshift
    GP16 - CAN RX Pin - to SO (MISO)
    GP17 - CAN CS Pin
    GP18 - CAN SCK Pin
    GP19 - CAN TX Pin - to SI (MOSI)
    GP20 - CAN Interrupt Pin

    Ref: https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg

 —————————————————————————————————————————————————————————————————————————————— */

#include "DFRobot_MCP2515.h"
#include "driver/twai.h"

#define upSW 15
#define downSW 14
int shiftval = 0;

#define RX_PIN 21
#define TX_PIN 22
// Intervall:
#define POLLING_RATE_MS 100
#define TRANSMIT_RATE_MS 100

static bool driver_installed = false;
unsigned long previousMillis = 0; // will store last time a message was send

void setup()
{
    Serial.begin(115200);
    delay(1000);
    pinMode(upSW, INPUT_PULLUP);
    pinMode(downSW, INPUT_PULLUP);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Look in the api-reference for other speed sets.
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        Serial.println("Driver installed");
    }
    else
    {
        Serial.println("Failed to install driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        Serial.println("Driver started");
    }
    else
    {
        Serial.println("Failed to start driver");
        return;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
    {
        Serial.println("CAN Alerts reconfigured");
    }
    else
    {
        Serial.println("Failed to reconfigure alerts");
        return;
    }

    // TWAI driver is now successfully installed and started
    driver_installed = true;
    delay(1000);
}

static void send_message()
{
    // Send message

    // Configure message to transmit
    twai_message_t message;
    message.identifier = 0x0F6;
    message.data_length_code = 4;
    for (int i = 0; i < 4; i++)
    {
        message.data[i] = 0;
    }

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        printf("Message queued for transmission\n");
    }
    else
    {
        printf("Failed to queue message for transmission\n");
    }
}

static void handle_rx_message(twai_message_t &message)
{
    // Process received message
    if (message.extd)
    {
        Serial.println("Message is in Extended Format");
    }
    else
    {
        Serial.println("Message is in Standard Format");
    }
    Serial.printf("ID: %x\nByte:", message.identifier);
    if (!(message.rtr))
    {
        for (int i = 0; i < message.data_length_code; i++)
        {
            Serial.printf(" %d = %02x,", i, message.data[i]);
        }
        Serial.println("");
    }
}

void shiftMode()
{
    int upval = digitalRead(upSW);
    int downval = digitalRead(downSW);
    if (upval == 1 && downval == 0)
    {
        shiftval = 1;
        delay(60);
    }
    else if (upval == 0 && downval == 1)
    {
        shiftval = 2;
        delay(60);
    }
    else if (upval == downval)
    {
        delay(60);
    }
}

static char buff[100];
unsigned char up[8] = {0, 1, 1, 0, 0, 0, 0, 0};
unsigned char down[8] = {0, 1, 2, 0, 0, 0, 0, 0};
unsigned char pidchk[8] = {2, 1, 0, 0, 0, 0, 0, 0};
unsigned char milClr[8] = {2, 1, 1, 0, 0, 0, 0, 0};
unsigned char coolTemp[8] = {2, 1, 5, 0, 0, 0, 0, 0};
unsigned char rpmm[8] = {2, 1, 12, 0, 0, 0, 0, 0};
unsigned char ambtemp[8] = {2, 1, 70, 0, 0, 0, 0, 0};

void loop()
{
    if (!driver_installed)
    {
        // Driver not installed
        delay(1000);
        return;
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    shiftMode();
    switch (shiftval)
    {
    case 1:

        delay(400);
        break;
    case 2:

        delay(400);
        break;

    default:
        delay(100);
        shiftval = 0;
        break;
    }
    shiftval = 0;
    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
        Serial.println("Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
        Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        Serial.println("Alert: The Transmission failed.");
        Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
        Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
        Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
    {
        Serial.println("Alert: The Transmission was successful.");
        Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    // Send message
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS)
    {
        previousMillis = currentMillis;
        send_message();
    }
    // char Order = Serial.read();
    // if (Order == '1')
    // {
    //     CAN.sendMsgBuf(0x02, 0, 8, pidchk);
    //     Order = 0;
    // }
    // else if (Order == '2')
    // {
    //     CAN.sendMsgBuf(0x02, 0, 8, milClr);
    //     Order = 0;
    // }
    // else if (Order == '3')
    // {
    //     CAN.sendMsgBuf(0x02, 0, 8, coolTemp);
    //     Order = 0;
    // }
    // else if (Order == '4')
    // {
    //     CAN.sendMsgBuf(0x02, 0, 8, rpmm);
    //     Order = 0;
    // }
    // else if (Order == '5')
    // {
    //     CAN.sendMsgBuf(0x02, 0, 8, ambtemp);
    //     Order = 0;
    // }
    // else if (Order == 0)
    // {
    // }
    // static int p;
    // char b[90];

    // if (flagRecv)
    // { // check if get data

    //     flagRecv = 0; // clear flag

    //     // iterate over all pending messages
    //     // If either the bus is saturated or the MCU is busy,
    //     // both RX buffers may be in use and after having read a single
    //     // message, MCU does  clear the corresponding IRQ conditon.
    //     while (CAN_MSGAVAIL == CAN.checkReceive())
    //     {
    //         // read data,  len: data length, buf: data buf
    //         CAN.readMsgBuf(&len, buf);
    //         canId = CAN.getCanId();
    //         if (canId == 0x7E8)
    //         {
    //             if (buf[0] == 4 && buf[1] == 65)
    //             {
    //                 switch (buf[2])
    //                 {
    //                 case 70:
    //                     Serial.print("Ambient Temperature: ");
    //                     Serial.println(buf[3]);
    //                     break;
    //                 case 63:
    //                     Serial.println("Status Cleared.");
    //                     break;
    //                 case 5:
    //                     Serial.print("Coolant Temperature: ");
    //                     Serial.println(buf[3], OCT);
    //                     break;
    //                 case 12:
    //                     Serial.print("TachoMeter: ");
    //                     Serial.println(buf[3], OCT);
    //                     break;

    //                 default:
    //                     break;
    //                 }
    //             }
    //         }
    //         else if (canId)
    //             // print the data
    //             for (int i = 0; i < len; i++)
    //             {
    //                 BuildMessage = BuildMessage + buf[i] + ",";
    //             }
    //         Serial.println(BuildMessage);
    //         BuildMessage = "";
    //         Serial.println(" ");
    //         delay(100);
    //     }
    // }
    delay(100);
}
