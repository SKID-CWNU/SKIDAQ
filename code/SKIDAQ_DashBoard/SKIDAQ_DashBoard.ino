/* ——————————————————————————————————————————————————————————————————————————————
    SKIDAQ Steering Wheel Dashboard
    Espressif ESP32 based SKID Datalogger Display Module
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
——————————————————————————————————————————————————————————————————————————————*/
#include "SKIDAQ_DashBoard.h"
#include <Wire.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <core_version.h>
#include "lvgl.h"
#include "ui.h"
#include "lv_conf.h"
#include "driver/twai.h"

#define USE_UI // if you want to use the ui export from Squareline, please do not annotate this line.
#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

// Macros
#define SERIAL_BAUD 115200
#define drs 17
#define LVGL_REFRESH_TIME (5u) // 5 milliseconds
static uint32_t lvgl_refresh_timestamp = 0u;

#define RX_PIN 37
#define TX_PIN 38
#define POLLING_RATE_MS 1000
static bool driver_installed = false;

// Condition Values Initialize
int rpmVal = 0;
int spVal = 0;
int drsVal = 0;
int tempVal = 0;
uint32_t i = 0;

extern lv_obj_t *ui_Spinbox1;
extern lv_obj_t *ui_Spinbox2;
extern lv_obj_t *ui_Spinbox3;
extern lv_obj_t *ui_Label2;

/*******************************************************************************
 * Screen Driver Configuration
 *******************************************************************************/

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
    bus,
    480 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 43 /* hsync_back_porch */,
    272 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 12 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 7000000 /* prefer_speed */, true /* auto_flush */);

/*******************************************************************************
 * Screen Driver Configuration  end
 *******************************************************************************/

#include "touch.h"
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[480 * 272 / 10]; // 5,7inch: lv_color_t disp_draw_buf[800*480/10]            4.3inch: lv_color_t disp_draw_buf[480*272/10]
// static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  lcd->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  lcd->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.setDebugOutput(true);
  //--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
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

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%u Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(35, OUTPUT);
  digitalWrite(35, LOW);
  pinMode(drs, INPUT_PULLUP);
  // digitalWrite(18, LOW);
  pinMode(0, OUTPUT); // TOUCH-CS

  // Init Display
  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);
  delay(200);
  lv_init();
  delay(100);
  touch_init();

  screenWidth = lcd->width();
  screenHeight = lcd->height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);
  //  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 480 * 272 / 10);
  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  ui_init(); // ui from Squareline or GUI Guider
  Serial.println("Setup done");
  delay(3000);
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

void loop()
{
  drsVal = digitalRead(drs);
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
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
  {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA)
  {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK)
    {
      handle_rx_message(message);
    }
  }
  lv_timer_handler();
  lv_spinbox_set_value(ui_Spinbox2, spVal);

  if (drsVal == 1)
  {
    lv_label_set_text(ui_Label2, "ON");
  }
  else if (drsVal == 0)
  {
    lv_label_set_text(ui_Label2, "OFF");
  }
  else
  {
    lv_label_set_text(ui_Label2, "N/A");
  }
  delay(100);
}

Sensor_Data_s * Get_TemperatureAndHumidity( void )
{
  return &sensor_data;
}