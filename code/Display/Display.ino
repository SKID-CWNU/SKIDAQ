#include <Wire.h>
#include <SPI.h>

#define USE_UI // if you want to use the ui export from Squareline, please do not annotate this line.

#if defined USE_UI
#include "lvgl.h"
#include "ui.h"
#include "lv_conf.h"
#endif
unsigned long watch_tick;

#include <Arduino_GFX_Library.h>
#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

#define SERIAL_BAUD 115200
#define I2C_DEV_ADDR 0x55

extern lv_obj_t * ui_Spinbox1;
extern lv_obj_t * ui_Spinbox2;
extern lv_obj_t * ui_Spinbox3;
int rpmVal = 0;
int spVal = 0;
int coolVal = 0;
int tempVal = 0;
uint32_t i = 0;
#define sda 17
#define scl 18

/******Please define a corresponding line based on your development board.************/
#define Display_43
// #define Display_50
// #define Display_70
/*******************************************************************************
 * Screen Driver Configuration
 *******************************************************************************/
#if defined(Display_43) // 4.3INCH 480x272
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

#elif defined(Display_50) // 5.0INCH 800x480
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 0 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
    bus,
    800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 43 /* hsync_back_porch */,
    480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 12 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#elif defined(Display_70) // 7.0INCH 800x480
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 0 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
    bus,
    800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 1 /* hsync_pulse_width */, 46 /* hsync_back_porch */,
    480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 23 /* vsync_back_porch */,
    0 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
#endif

/*******************************************************************************
 * Screen Driver Configuration  end
 *******************************************************************************/

/*******************************************************************************
   Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

#ifdef USE_UI
/* Change to your screen resolution */
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

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print("Data x :");
      Serial.println(touch_last_x);

      Serial.print("Data y :");
      Serial.println(touch_last_y);
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}
#endif

void onRequest()
{
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}
void onReceive(int len)
{
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available())
  {
    spVal = Wire.read();
    lv_spinbox_set_value(ui_Spinbox2, spVal);
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("LVGL Widgets Demo");
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.setPins(sda, scl);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if defined(Display_50) || defined(Display_70)
  // IO Port Pins
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
#elif defined(Display_43)
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(35, OUTPUT);
  digitalWrite(35, LOW);
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  pinMode(0, OUTPUT); // TOUCH-CS
#endif

  // Init Display
  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);
  delay(200);

#ifdef USE_UI
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
#endif

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

#ifdef USE_UI
  ui_init(); // ui from Squareline or GUI Guider
#else
  lcd->fillScreen(RED);
  delay(800);
  lcd->fillScreen(BLUE);
  delay(800);
  lcd->fillScreen(YELLOW);
  delay(800);
  lcd->fillScreen(GREEN);
  delay(800);
#endif
  Serial.println("Setup done");
}

void loop()
{

  while (1)
  {
#ifdef USE_UI
    lv_timer_handler();
    delay(5);
#endif
  }
}
