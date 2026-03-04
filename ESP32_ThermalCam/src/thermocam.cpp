#include "infobar.h"
#include "mlxcamera.h"
//TTGO:
#include <TFT_eSPI.h>
#include <Button2.h>

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif
#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif
#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23
#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35
#define BUTTON_2        0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

//char buff[512];
//int vref = 1100;
int btnCick = false;
//TTGO end

MLXCamera camera(tft);
InfoBar infoBar = InfoBar(tft);
const uint32_t InfoBarHeight = 10;
const uint32_t MaxFrameTimeInMillis = 33;

InterpolationType interpolationType = InterpolationType::eLinear;
//InterpolationType interpolationType = InterpolationType::eNone;
bool fixedTemperatureRange = true;

void setup() {
  tft.init();
  //tft.setRotation(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);

  if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }
  //tft.init();
  //tft.setRotation(3);
  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_DARKGREY);

  Serial.begin(115200);
  while(!Serial);

  if (!camera.init())
  {      
    tft.setCursor(75, tft.height() / 2);
    tft.print("No camera detected!");
    vTaskDelete(NULL); // remove loop task
  }

  camera.drawLegendGraph();
}

void loop() {
    const long start = millis();

    camera.readImage();

    const long processingTime = millis() - start;

    /*uint16_t dummyX = 0, dummyY = 0;
    if (tft.getTouch(&dummyX, &dummyY))
    {
      if (dummyX > 80)
        interpolationType++;
      else
      {
        fixedTemperatureRange = !fixedTemperatureRange;
        if (fixedTemperatureRange)
          camera.setFixedTemperatureRange();
        else
          camera.setDynamicTemperatureRange();
      }
    }*/

    //camera.setFixedTemperatureRange();
    camera.setDynamicTemperatureRange();
    
    tft.setCursor(0, InfoBarHeight);
    camera.drawImage(interpolationType);
    camera.drawLegendText();
    camera.drawCenterMeasurement();

    const long frameTime = millis() - start;

    infoBar.update(start, processingTime, frameTime);
    if (frameTime < MaxFrameTimeInMillis)
      delay(MaxFrameTimeInMillis - frameTime);
}
