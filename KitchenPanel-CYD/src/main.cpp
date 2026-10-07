// Kitchen panel for the CYD ESP32-3248S035C.
//
// Top: the house battery - a bar with the state of charge (V128), the voltage
// (V124) and the power (V129; the JK BMS reports it positive while charging and
// negative while discharging).
// Below: three sliders for the kitchen lights on LEDKitchen1 - sink (V45) and
// cooktop (V44) LED strips, 0..60, and the red hallway LED (V43), 0..10.
//
// Everything goes through the local legacy Blynk server's HTTP API with the
// CAN bridge's token:
//   GET /<token>/get/V128              -> ["90"]
//   GET /<token>/update/V45?value=12   -> forwarded to the hardware and the app,
//                                         exactly like moving the app slider
// So the app, this panel and the LEDs always agree, and the bridge's own
// connection is never touched.
//
// The display and touch code is KeyLang's (C:\Users\nik\AI_Proj\KeyLang\esp32),
// proven on this board. Every touch flashes the RGB LED white, whatever else
// happens - it says "the GT911 was read" without depending on anything above it.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>

#include "secrets.h"

// ---------------------------------------------------------------------------
// BOARD CONFIG (from KeyLang, confirmed on the actual board)
// ---------------------------------------------------------------------------
static constexpr int kPinTftSclk = 14;
static constexpr int kPinTftMosi = 13;
static constexpr int kPinTftMiso = 12;
static constexpr int kPinTftCs   = 15;
static constexpr int kPinTftDc   = 2;
static constexpr int kPinTftRst  = -1;
static constexpr int kPinBacklight = 27;

static constexpr int kPinTouchSda = 33;
static constexpr int kPinTouchScl = 32;
static constexpr int kPinTouchInt = 21;
static constexpr int kPinTouchRst = 25;
static constexpr uint8_t kGt911Addresses[] = {0x5D, 0x14};

// Common-anode RGB LED: driving a pin LOW lights that channel.
static constexpr int kPinLedRed   = 4;
static constexpr int kPinLedGreen = 16;
static constexpr int kPinLedBlue  = 17;

static constexpr int kPanelWidth = 320;   // native portrait
static constexpr int kPanelHeight = 480;

// Touch orientation for setRotation(1): bit 0 swaps axes, bit 1 flips x, bit 2
// flips y. KeyLang uses MAP 1, but it only ever reads x (three columns), so its
// inverted y went unnoticed; on this board's sliders it showed: 1|4 = 5.
static constexpr uint8_t kMapMode = 5;

// Backlight 100 of 255: the panel draws a lot and browns out on weak USB.
static constexpr uint8_t kBrightness = 100;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static constexpr uint32_t kPollSpacingMs = 400;    // one pin per step, 6 pins
static constexpr uint32_t kSendSpacingMs = 250;    // while dragging
static constexpr uint32_t kHoldAfterSendMs = 3000; // ignore polls of a pin we just set
static constexpr uint32_t kStaleMs = 15000;        // grey out values older than this
static constexpr uint16_t kHttpTimeoutMs = 1200;
static constexpr uint32_t kTouchFlashMs = 50;

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

class CydDisplay : public lgfx::LGFX_Device {
  // No lgfx touch driver: its GT911 backend never reported a contact on this
  // board, so the touch is read directly below (see KeyLang).
  lgfx::Panel_ST7796 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;

 public:
  CydDisplay() {
    {
      auto cfg = _bus.config();
      cfg.spi_host = HSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = kPinTftSclk;
      cfg.pin_mosi = kPinTftMosi;
      cfg.pin_miso = kPinTftMiso;
      cfg.pin_dc = kPinTftDc;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = kPinTftCs;
      cfg.pin_rst = kPinTftRst;
      cfg.pin_busy = -1;
      cfg.panel_width = kPanelWidth;
      cfg.panel_height = kPanelHeight;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;   // the SD slot shares this SPI bus
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl = kPinBacklight;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static CydDisplay gfx;
// Off-screen buffers: everything that changes is drawn here and pushed whole,
// so the screen never shows a cleared area between two frames.
static LGFX_Sprite gBarSprite(&gfx);
static LGFX_Sprite gRowSprite(&gfx);
static LGFX_Sprite gLineSprite(&gfx);   // voltage/power line, also the status line

// ---------------------------------------------------------------------------
// Colours
// ---------------------------------------------------------------------------
static constexpr uint32_t kBg = 0x000000;
static constexpr uint32_t kPanelBg = 0x141414;
static constexpr uint32_t kText = 0xFFFFFF;
static constexpr uint32_t kDimText = 0x9E9E9E;
static constexpr uint32_t kStaleText = 0x5A5A5A;
static constexpr uint32_t kTrack = 0x3A3A3A;
static constexpr uint32_t kSocRed = 0xE53935;
static constexpr uint32_t kSocOrange = 0xFB8C00;
static constexpr uint32_t kSocGreen = 0x43A047;
static constexpr uint32_t kCharging = 0x66BB6A;
static constexpr uint32_t kDischarging = 0xFFA726;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

struct Slider {
  const char* label;
  uint8_t vpin;
  int maxValue;
  uint32_t colour;
  int value;               // what the panel shows
  bool known;              // value has been read at least once
  bool dirty;              // changed here, not yet sent
  uint32_t lastSend;
  uint32_t holdUntil;      // polls of this pin are ignored until then
};

static Slider gSliders[] = {
    {"Sink",    45, 60, 0xFFE082, 0, false, false, 0, 0},
    {"Cooktop", 44, 60, 0xFFCC80, 0, false, false, 0, 0},
    {"Hallway", 43, 10, 0xE53935, 0, false, false, 0, 0},
};
static constexpr int kSliderCount = sizeof(gSliders) / sizeof(gSliders[0]);

struct Battery {
  float soc = NAN;    // %
  float volts = NAN;  // V
  float watts = NAN;  // W, + charging, - discharging
  uint32_t socAt = 0, voltsAt = 0, wattsAt = 0;
};
static Battery gBat;

static bool gServerOk = false;
static uint32_t gLastServerOk = 0;
static uint8_t gTouchAddr = 0;
static uint32_t gFlashUntil = 0;
static int gDragging = -1;          // slider index while a finger is on it

// ---------------------------------------------------------------------------
// Layout (landscape 480x320)
// ---------------------------------------------------------------------------
static constexpr int kBarX = 16, kBarY = 12, kBarW = 440, kBarH = 70;
static constexpr int kInfoY = 92, kInfoH = 34;
static constexpr int kRowsY = 136, kRowH = 54;
static constexpr int kLabelW = 128;                     // label column
static constexpr int kRowSpriteX = kLabelW;             // sprite covers the rest
static constexpr int kTrackX = 18, kTrackW = 250;       // inside the row sprite
static constexpr int kStatusY = 296, kStatusH = 24;

// ---------------------------------------------------------------------------
// RGB LED
// ---------------------------------------------------------------------------

static void ledSetup() {
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(kPinLedRed, 0);
  ledcAttachPin(kPinLedGreen, 1);
  ledcAttachPin(kPinLedBlue, 2);
}

// Common anode: full brightness is duty 0, off is duty 255.
static void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(0, 255 - r);
  ledcWrite(1, 255 - g);
  ledcWrite(2, 255 - b);
}

// Idle LED: off while all is well, dim red while the server is unreachable.
static void ledIdle() {
  if (gServerOk) ledWrite(0, 0, 0);
  else ledWrite(25, 0, 0);
}

// ---------------------------------------------------------------------------
// GT911 (direct register access, as in KeyLang)
// ---------------------------------------------------------------------------
static constexpr uint16_t kGtRegStatus = 0x814E;
static constexpr uint16_t kGtRegPoint1 = 0x8150;

static uint8_t probeTouchController() {
  Wire.begin(kPinTouchSda, kPinTouchScl, 400000);
  // The GT911 latches its address from INT during reset; INT low selects 0x5D.
  pinMode(kPinTouchRst, OUTPUT);
  pinMode(kPinTouchInt, OUTPUT);
  digitalWrite(kPinTouchRst, LOW);
  digitalWrite(kPinTouchInt, LOW);
  delay(12);
  digitalWrite(kPinTouchRst, HIGH);
  delay(60);
  pinMode(kPinTouchInt, INPUT);
  delay(60);
  for (uint8_t addr : kGt911Addresses) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return addr;
  }
  return 0;
}

static bool gtRead(uint16_t reg, uint8_t* out, size_t len) {
  if (gTouchAddr == 0) return false;
  Wire.beginTransmission(gTouchAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)gTouchAddr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool gtWrite(uint16_t reg, uint8_t value) {
  if (gTouchAddr == 0) return false;
  Wire.beginTransmission(gTouchAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

enum class TouchFrame { None, Down, Up };

// KeyLang only needed taps; a slider needs to know when the finger lifts, so
// a frame with zero contacts is reported as Up rather than ignored.
static TouchFrame gtPoll(int32_t* nx, int32_t* ny) {
  uint8_t status;
  if (!gtRead(kGtRegStatus, &status, 1)) return TouchFrame::None;
  if ((status & 0x80) == 0) return TouchFrame::None;   // no new frame

  TouchFrame result = TouchFrame::Up;
  if ((status & 0x0F) > 0) {
    uint8_t d[8];
    if (gtRead(kGtRegPoint1, d, sizeof(d))) {
      *nx = (int32_t)(d[0] | (d[1] << 8));
      *ny = (int32_t)(d[2] | (d[3] << 8));
      result = TouchFrame::Down;
    } else {
      result = TouchFrame::None;
    }
  }
  gtWrite(kGtRegStatus, 0);   // must clear, always, or the chip stops reporting
  return result;
}

static void gtMapToScreen(int32_t nx, int32_t ny, int32_t* sx, int32_t* sy) {
  const bool swap = (kMapMode & 0x1) != 0;
  int32_t ax = swap ? ny : nx;
  int32_t ay = swap ? nx : ny;
  int32_t axMax = swap ? (kPanelHeight - 1) : (kPanelWidth - 1);
  int32_t ayMax = swap ? (kPanelWidth - 1) : (kPanelHeight - 1);
  if (kMapMode & 0x2) ax = axMax - ax;
  if (kMapMode & 0x4) ay = ayMax - ay;
  *sx = ax * gfx.width() / (axMax + 1);
  *sy = ay * gfx.height() / (ayMax + 1);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

static bool fresh(uint32_t at) { return at != 0 && millis() - at < kStaleMs; }

static uint32_t socColour(float soc) {
  if (soc < 20) return kSocRed;
  if (soc < 40) return kSocOrange;
  return kSocGreen;
}

static void drawBattery() {
  LGFX_Sprite& s = gBarSprite;
  const bool known = !isnan(gBat.soc);
  const bool ok = known && fresh(gBat.socAt);

  // Redraw only when what is shown would change.
  static int lastSoc = -2;
  static int lastOk = -1;
  int socShown = known ? (int)(constrain(gBat.soc, 0.0f, 100.0f) + 0.5f) : -1;
  if (socShown == lastSoc && (int)ok == lastOk) return;
  lastSoc = socShown;
  lastOk = ok;

  s.fillScreen(kBg);
  // Body and terminal nub.
  s.drawRoundRect(0, 0, kBarW - 12, kBarH, 8, ok ? kText : kStaleText);
  s.drawRoundRect(1, 1, kBarW - 14, kBarH - 2, 7, ok ? kText : kStaleText);
  s.fillRoundRect(kBarW - 11, kBarH / 2 - 14, 10, 28, 3, ok ? kText : kStaleText);

  if (known) {
    int inner = kBarW - 12 - 10;
    int w = inner * socShown / 100;
    if (w > 0) s.fillRoundRect(5, 5, w, kBarH - 10, 5, ok ? socColour(socShown) : kTrack);
  }

  char text[12];
  if (known) snprintf(text, sizeof(text), "%d%%", socShown);
  else snprintf(text, sizeof(text), "--%%");
  s.setFont(&fonts::FreeSansBold24pt7b);
  s.setTextDatum(textdatum_t::middle_center);
  // A dark outline keeps the number readable over both the fill and the empty part.
  s.setTextColor(0x000000);
  for (int dx = -2; dx <= 2; dx += 2)
    for (int dy = -2; dy <= 2; dy += 2)
      s.drawString(text, (kBarW - 12) / 2 + dx, kBarH / 2 + dy);
  s.setTextColor(ok ? kText : kDimText);
  s.drawString(text, (kBarW - 12) / 2, kBarH / 2);

  s.pushSprite(kBarX, kBarY);
}

static void drawBatteryInfo() {
  // Voltage, left.
  char volts[16];
  if (!isnan(gBat.volts)) snprintf(volts, sizeof(volts), "%.2f V", gBat.volts);
  else snprintf(volts, sizeof(volts), "-- V");
  uint32_t voltsColour = fresh(gBat.voltsAt) ? kText : kStaleText;

  // Power, right: + charging (green, up arrow), - discharging (orange, down).
  char power[32];
  uint32_t colour = kDimText;
  int arrow = 0;   // +1 up, -1 down, 0 none
  if (isnan(gBat.watts)) {
    snprintf(power, sizeof(power), "-- W");
  } else {
    int w = (int)lroundf(gBat.watts);
    if (w >= 5) {
      snprintf(power, sizeof(power), "+%d W charging", w);
      colour = kCharging;
      arrow = 1;
    } else if (w <= -5) {
      snprintf(power, sizeof(power), "%d W discharging", w);
      colour = kDischarging;
      arrow = -1;
    } else {
      snprintf(power, sizeof(power), "%d W idle", w);
    }
  }
  if (!fresh(gBat.wattsAt)) colour = kStaleText;

  // Redraw only when the visible text or colours change.
  static char lastVolts[16] = "";
  static char lastPower[32] = "";
  static uint32_t lastVoltsColour = 1, lastColour = 1;
  if (strcmp(volts, lastVolts) == 0 && strcmp(power, lastPower) == 0 &&
      voltsColour == lastVoltsColour && colour == lastColour) {
    return;
  }
  strcpy(lastVolts, volts);
  strcpy(lastPower, power);
  lastVoltsColour = voltsColour;
  lastColour = colour;

  LGFX_Sprite& s = gLineSprite;
  const int cy = kInfoH / 2;
  s.fillScreen(kBg);
  s.setFont(&fonts::Font4);

  s.setTextDatum(textdatum_t::middle_left);
  s.setTextColor(voltsColour);
  s.drawString(volts, kBarX, cy);

  const int right = kBarX + kBarW - 12;
  s.setTextDatum(textdatum_t::middle_right);
  s.setTextColor(colour);
  int textW = s.textWidth(power);
  s.drawString(power, right, cy);

  if (arrow != 0) {
    int ax = right - textW - 18;
    if (arrow > 0) s.fillTriangle(ax - 9, cy + 7, ax + 9, cy + 7, ax, cy - 9, colour);
    else s.fillTriangle(ax - 9, cy - 7, ax + 9, cy - 7, ax, cy + 9, colour);
  }

  s.pushSprite(0, kInfoY);
}

static int rowTop(int i) { return kRowsY + i * kRowH; }

static void drawSliderLabel(int i) {
  const int y = rowTop(i);
  gfx.fillRect(0, y, kLabelW, kRowH, kBg);
  gfx.setFont(&fonts::Font4);
  gfx.setTextDatum(textdatum_t::middle_left);
  gfx.setTextColor(kText, kBg);
  gfx.drawString(gSliders[i].label, kBarX, y + kRowH / 2);
}

static void drawSlider(int i) {
  const Slider& sl = gSliders[i];
  LGFX_Sprite& s = gRowSprite;
  const int cy = kRowH / 2;
  const bool live = sl.known && (gServerOk || sl.dirty || gDragging == i);

  s.fillScreen(kBg);
  s.fillRoundRect(kTrackX, cy - 6, kTrackW, 12, 6, kTrack);

  if (sl.known) {
    int kx = kTrackX + (int)((long)kTrackW * sl.value / sl.maxValue);
    uint32_t c = live ? sl.colour : kStaleText;
    if (kx > kTrackX) s.fillRoundRect(kTrackX, cy - 6, kx - kTrackX, 12, 6, c);
    s.fillCircle(kx, cy, gDragging == i ? 17 : 14, c);
    s.drawCircle(kx, cy, gDragging == i ? 17 : 14, kText);
  }

  char text[8];
  if (sl.known) snprintf(text, sizeof(text), "%d", sl.value);
  else snprintf(text, sizeof(text), "--");
  s.setFont(&fonts::Font4);
  s.setTextDatum(textdatum_t::middle_right);
  s.setTextColor(live ? kText : kStaleText);
  s.drawString(text, s.width() - 14, cy);

  s.pushSprite(kRowSpriteX, rowTop(i));
}

static void drawStatus() {
  char line[96];
  if (WiFi.status() != WL_CONNECTED) {
    snprintf(line, sizeof(line), "Wi-Fi: connecting to %s ...", WIFI_SSID);
  } else if (!gServerOk) {
    snprintf(line, sizeof(line), "Wi-Fi %s  |  Blynk %s:%d not answering",
             WiFi.localIP().toString().c_str(), BLYNK_HOST, BLYNK_PORT);
  } else {
    snprintf(line, sizeof(line), "Wi-Fi %s  %d dBm  |  Blynk OK",
             WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  static char lastLine[96] = "\x01";
  if (strcmp(line, lastLine) == 0) return;
  strcpy(lastLine, line);

  // Shares the line sprite; it is taller than the status strip, and the part
  // below the screen edge is simply clipped.
  LGFX_Sprite& s = gLineSprite;
  const int cy = kStatusH / 2;
  s.fillScreen(kPanelBg);
  s.setFont(&fonts::Font2);
  s.setTextDatum(textdatum_t::middle_left);
  s.setTextColor(kDimText);
  s.drawString(line, 8, cy);
  if (gTouchAddr == 0) {
    s.setTextDatum(textdatum_t::middle_right);
    s.setTextColor(kSocRed);
    s.drawString("TOUCH NOT FOUND", s.width() - 8, cy);
  }
  s.pushSprite(0, kStatusY);
}

static void redrawAll() {
  gfx.fillScreen(kBg);
  drawBattery();
  drawBatteryInfo();
  for (int i = 0; i < kSliderCount; i++) {
    drawSliderLabel(i);
    drawSlider(i);
  }
  drawStatus();
}

// ---------------------------------------------------------------------------
// Blynk HTTP API
// ---------------------------------------------------------------------------

static WiFiClient gNet;
static HTTPClient gHttp;

static void setServerOk(bool ok) {
  if (ok) gLastServerOk = millis();
  if (ok != gServerOk) {
    gServerOk = ok;
    ledIdle();
    drawStatus();
    for (int i = 0; i < kSliderCount; i++) drawSlider(i);
  }
}

// GET <path>; returns the HTTP status (or a negative HTTPClient error).
static int httpGet(const String& path, String* body) {
  gHttp.setReuse(true);
  gHttp.setTimeout(kHttpTimeoutMs);
  gHttp.setConnectTimeout(kHttpTimeoutMs);
  if (!gHttp.begin(gNet, BLYNK_HOST, BLYNK_PORT, path)) return -1;
  int code = gHttp.GET();
  if (code == 200 && body) *body = gHttp.getString();
  gHttp.end();
  return code;
}

// /get/Vn answers ["12.5"]. Anything else (e.g. "Invalid token.") is a failure.
static bool blynkGet(uint8_t vpin, float* value) {
  String body;
  int code = httpGet(String("/") + BLYNK_TOKEN + "/get/V" + vpin, &body);
  if (code < 0) {
    setServerOk(false);
    return false;
  }
  setServerOk(true);   // the server answered; the pin may still be empty
  if (code != 200) return false;
  int a = body.indexOf('"');
  int b = body.indexOf('"', a + 1);
  if (a < 0 || b <= a + 1) return false;
  String v = body.substring(a + 1, b);
  char* end = nullptr;
  float f = strtof(v.c_str(), &end);
  if (end == v.c_str()) return false;
  *value = f;
  return true;
}

static bool blynkUpdate(uint8_t vpin, int value) {
  int code = httpGet(String("/") + BLYNK_TOKEN + "/update/V" + vpin + "?value=" + value, nullptr);
  setServerOk(code > 0);
  Serial.printf("update V%u=%d -> %d\n", vpin, value, code);
  return code == 200;
}

// One pin per call, round-robin: a slow answer never stalls the touch for long.
static void pollNext() {
  static int step = 0;
  const int kSteps = 3 + kSliderCount;
  int s = step;
  step = (step + 1) % kSteps;

  float v;
  switch (s) {
    case 0:
      if (blynkGet(128, &v)) { gBat.soc = v; gBat.socAt = millis(); }
      drawBattery();
      break;
    case 1:
      if (blynkGet(124, &v)) { gBat.volts = v; gBat.voltsAt = millis(); }
      drawBatteryInfo();
      break;
    case 2:
      if (blynkGet(129, &v)) { gBat.watts = v; gBat.wattsAt = millis(); }
      drawBatteryInfo();
      break;
    default: {
      int i = s - 3;
      Slider& sl = gSliders[i];
      // Never let a poll fight a finger or a value still on its way.
      if (gDragging == i || sl.dirty || (int32_t)(millis() - sl.holdUntil) < 0) break;
      if (blynkGet(sl.vpin, &v)) {
        int nv = constrain((int)lroundf(v), 0, sl.maxValue);
        if (!sl.known || nv != sl.value) {
          sl.value = nv;
          sl.known = true;
          drawSlider(i);
        }
      }
      break;
    }
  }
}

static void sendPending(bool force) {
  for (int i = 0; i < kSliderCount; i++) {
    Slider& sl = gSliders[i];
    if (!sl.dirty) continue;
    if (!force && millis() - sl.lastSend < kSendSpacingMs) continue;
    if (WiFi.status() != WL_CONNECTED) continue;
    sl.lastSend = millis();
    if (blynkUpdate(sl.vpin, sl.value)) sl.dirty = false;
    sl.holdUntil = millis() + kHoldAfterSendMs;
  }
}

// ---------------------------------------------------------------------------
// Touch -> sliders
// ---------------------------------------------------------------------------

static int sliderAt(int32_t sy) {
  for (int i = 0; i < kSliderCount; i++) {
    if (sy >= rowTop(i) && sy < rowTop(i) + kRowH) return i;
  }
  return -1;
}

static int valueAt(const Slider& sl, int32_t sx) {
  int32_t x = sx - kRowSpriteX - kTrackX;
  long v = lroundf((float)x * sl.maxValue / kTrackW);
  return constrain((int)v, 0, sl.maxValue);
}

static void pumpTouch() {
  static uint32_t lastPoll = 0;
  static uint32_t lastFrame = 0;
  if (millis() - lastPoll < 15) return;   // the GT911 frame rate is far lower
  lastPoll = millis();

  int32_t nx = 0, ny = 0;
  TouchFrame f = gtPoll(&nx, &ny);

  // A held finger keeps producing frames; if they stop, treat it as lifted.
  bool lifted = (f == TouchFrame::Up) ||
                (f == TouchFrame::None && gDragging >= 0 && millis() - lastFrame > 200);

  if (f == TouchFrame::Down) {
    lastFrame = millis();
    int32_t sx, sy;
    gtMapToScreen(nx, ny, &sx, &sy);

    if (gDragging < 0) {
      // New contact: unconditional white blip, then pick the row it landed on.
      ledWrite(255, 255, 255);
      gFlashUntil = millis() + kTouchFlashMs;
      Serial.printf("# TOUCH raw=%ld,%ld map=%ld,%ld\n", (long)nx, (long)ny, (long)sx, (long)sy);
      int i = sliderAt(sy);
      if (i >= 0 && gSliders[i].known && sx >= kRowSpriteX) {
        gDragging = i;
      }
    }
    if (gDragging >= 0) {
      Slider& sl = gSliders[gDragging];
      int nv = valueAt(sl, sx);
      if (nv != sl.value) {
        sl.value = nv;
        sl.dirty = true;
      }
      drawSlider(gDragging);
    }
  } else if (lifted && gDragging >= 0) {
    int i = gDragging;
    gDragging = -1;
    sendPending(true);   // final value goes out at once
    drawSlider(i);
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  ledSetup();
  ledWrite(0, 0, 0);

  gTouchAddr = probeTouchController();
  if (gTouchAddr) gtWrite(kGtRegStatus, 0);

  gfx.init();
  gfx.setRotation(1);   // landscape 480x320
  gfx.setBrightness(kBrightness);

  // Sprites before Wi-Fi, while the heap is still in one piece.
  gBarSprite.setColorDepth(16);
  if (!gBarSprite.createSprite(kBarW, kBarH)) {
    gBarSprite.setColorDepth(8);
    gBarSprite.createSprite(kBarW, kBarH);
  }
  gRowSprite.setColorDepth(16);
  if (!gRowSprite.createSprite(480 - kRowSpriteX, kRowH)) {
    gRowSprite.setColorDepth(8);
    gRowSprite.createSprite(480 - kRowSpriteX, kRowH);
  }
  gLineSprite.setColorDepth(16);
  if (!gLineSprite.createSprite(480, kInfoH)) {
    gLineSprite.setColorDepth(8);
    gLineSprite.createSprite(480, kInfoH);
  }
  Serial.printf("GT911 0x%02X, heap %u, sprites %d/%d/%d bit\n", gTouchAddr,
                ESP.getFreeHeap(), gBarSprite.getColorDepth(), gRowSprite.getColorDepth(),
                gLineSprite.getColorDepth());

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // keeps HTTP latency low and steady
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  redrawAll();
  ledIdle();
}

void loop() {
  static uint32_t lastPoll = 0;
  static uint32_t lastStatus = 0;
  static wl_status_t lastWifi = WL_IDLE_STATUS;

  pumpTouch();

  if (gFlashUntil != 0 && (int32_t)(millis() - gFlashUntil) >= 0) {
    gFlashUntil = 0;
    ledIdle();
  }

  wl_status_t wifi = WiFi.status();
  if (wifi != lastWifi) {
    lastWifi = wifi;
    Serial.printf("Wi-Fi status %d %s\n", wifi, WiFi.localIP().toString().c_str());
    drawStatus();
  }

  if (wifi == WL_CONNECTED) {
    sendPending(false);
    // Polling pauses while a finger is on a slider: the drag stays smooth and
    // the only HTTP traffic is the value being set.
    if (gDragging < 0 && millis() - lastPoll >= kPollSpacingMs) {
      lastPoll = millis();
      pollNext();
    }
  } else if (gServerOk) {
    setServerOk(false);
  }

  // Refresh the status line (RSSI) and grey out stale values now and then.
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    drawStatus();
    if (gServerOk && millis() - gLastServerOk > kStaleMs) setServerOk(false);
  }

  delay(2);
}
