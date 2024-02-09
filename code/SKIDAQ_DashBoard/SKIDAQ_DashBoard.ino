// ——————————————————————————————————————————————————————————————————————————————//
//                        SKIDAQ Steering Wheel Dashboard                        //
//                  Raspberry Pi Pico based SKID-FS DAQ System                   //
//                             Author: Lim ChaeWon                               //
//            SKIDAQ © 2024 by Lim ChaeWon is licensed under GPL 3.0             //
// ——————————————————————————————————————————————————————————————————————————————//

#include <Wire.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <ACAN_ESP32.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <core_version.h>
#include "lvgl.h"
#include "ui.h"
#include "lv_conf.h"

#define USE_UI // if you want to use the ui export from Squareline, please do not annotate this line.

#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

#define SERIAL_BAUD 115200
#define rx 18
#define tx 17
#define drs 38

int rpmVal = 0;
int spVal = 0;
int drsVal = 0;
int tempVal = 0;
uint32_t i = 0;
static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL; // 1 Mb/s

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
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.print("ESP32 Arduino Release: ");
  Serial.println(ARDUINO_ESP32_RELEASE);
  Serial.print("ESP32 Chip Revision: ");
  Serial.println(chip_info.revision);
  Serial.print("ESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print("ESP32 Flash: ");
  uint32_t size_flash_chip;
  esp_flash_get_size(NULL, &size_flash_chip);
  Serial.print(size_flash_chip / (1024 * 1024));
  Serial.print(" MB ");
  Serial.println(((chip_info.features & CHIP_FEATURE_EMB_FLASH) != 0) ? "(embeded)" : "(external)");
  Serial.print("APB CLOCK: ");
  Serial.print(APB_CLK_FREQ);
  Serial.println(" Hz");
  //--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode;
  settings.mRxPin = GPIO_NUM_17; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_18; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode == 0)
  {
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);
    Serial.print("RJW:                ");
    Serial.println(settings.mRJW);
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Distance            ");
    Serial.print(settings.ppmFromDesiredBitRate());
    Serial.println(" ppm");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
    Serial.println("Configuration OK!");
  }
  else
  {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }

  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(35, OUTPUT);
  digitalWrite(35, LOW);
  pinMode(drs, INPUT_PULLUP);
  // digitalWrite(38, LOW);
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

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0;
static uint32_t gSentFrameCount = 0;

void loop()
{
  drsVal = digitalRead(drs);
  CANMessage frame;
  if (gBlinkLedDate < millis())
  {
    gBlinkLedDate += 500;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.print("Sent: ");
    Serial.print(gSentFrameCount);
    Serial.print(" ");
    Serial.print("Receive: ");
    Serial.print(gReceivedFrameCount);
    Serial.print(" ");
    Serial.print(" STATUS 0x");
    Serial.print(TWAI_STATUS_REG, HEX);
    Serial.print(" RXERR ");
    Serial.print(TWAI_RX_ERR_CNT_REG);
    Serial.print(" TXERR ");
    Serial.println(TWAI_TX_ERR_CNT_REG);
    const bool ok = ACAN_ESP32::can.tryToSend(frame);
    if (ok)
    {
      gSentFrameCount += 1;
    }
  }
  while (ACAN_ESP32::can.receive(frame))
  {
    gReceivedFrameCount += 1;
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
