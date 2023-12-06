#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//#include <ESP8266WiFi.h> //gone to esp32
#include <PubSubClient.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <OpenTherm.h>

//TTGO:
#include <TFT_eSPI.h> 
//!uncomment line in: TFT_eSPI/User_Setup_Select.h
//  #include <User_Setups/Setup25_TTGO_T_Display.h>    // Setup file for ESP32 and TTGO T-Display ST7789V SPI bus TFT
#include <Button2.h>

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif
#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif
//#define TFT_MOSI            19
//#define TFT_SCLK            18
//#define TFT_CS              5
//#define TFT_DC              16
//#define TFT_RST             23
//#define TFT_BL              4  // Display backlight control pin

//#define TFT_WIDTH  135 //X
//#define TFT_HEIGHT 240 //Y

#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35 //right
#define BUTTON_2        0  //left

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

Button2 btn1(BUTTON_1);
//Button2 btn2(BUTTON_2);
int btnCick = false;
//TTGO end

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 12;
const int outPin = 13;

////Data wire is connected to 14 pin on the OpenTherm Shield
//#define ONE_WIRE_BUS 14

const char* ssid = "WLNA";
const char* password = "HKP35241";
const char* mqtt_server = "192.168.0.130";
const int   mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
OpenTherm OpenThermIf(inPin, outPin);
WiFiClient espClient;
PubSubClient MQTTClient(espClient);
char buf[10];

float CHSetPoint = 24; //set point
float CHCurTemp = 0; //current temperature
//pv_last = 0, //prior temperature
//ierr = 0, //integral error
//dt = 0, //time between measurements
//op = 0; //PID controller output
unsigned long lastOTSetTempMillis = 0, curMillis = 0;

long lastReconnectMQTTmillis=0;

void IRAM_ATTR handleInterrupt() { //was ICACHE_RAM_ATTR
  OpenThermIf.handleInterrupt();
}

void event_on_btn1(Button2& b) {
   //if (btn == buttonA) //for multiple buttons (one func)
  //if (b.wasPressedFor() > 1000) // check for really long clicks - subscribed by setTapHandler()
  
}

void setup_wifi() {
  delay(10);
  //Connect to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(10000);
    ESP.restart();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void MQTTpublish_temperature() {
  Serial.println("t=" + String(CHCurTemp));    
  String(CHCurTemp).toCharArray(buf, 10);
  MQTTClient.publish("/home1/OpenTherm/CHCurTemp", buf);  
}

void MQTTcallback(char* topic, byte* payload, uint length) {
  if(strcmp(topic, "CHSetPoint") != 0) return;
  String str = String();    
  for (uint i = 0; i < length; i++) {
    str += (char)payload[i];
  }
  Serial.println("CHSetPoint=" + str);  
  CHSetPoint = str.toFloat();
}

void MQTTReconnect() {  
  long curMillis = millis();
  if( curMillis - lastReconnectMQTTmillis < 5000 ){ 
    return; //try reconnect not too often - once in 5 sec.
  }
  lastReconnectMQTTmillis = curMillis;

  while (!MQTTClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTTClient.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      MQTTpublish_temperature();
      // ... and resubscribe
      MQTTClient.subscribe("CHSetPoint");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }    
  }
}

void setup_ArduinoOTA(){
  // Port defaults to 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32_TTGO1");

  // No authentication by default
  ArduinoOTA.setPassword("AUTHpwd_esp32");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA Done.");
}

void setup_TFT(){
  tft.init();
  //tft.setRotation(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLUE); //blink on boot
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM); //MC=Middle Centre
  //tft.setCursor(0, 0);
  //tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);

  if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  //tft.fillRect(10, 20, 20, 40, TFT_BLUE);
  tft.fillRectHGradient(0,0,TFT_HEIGHT,2,TFT_MAGENTA,TFT_BLUE);
  tft.fillRectHGradient(0,TFT_WIDTH-2,TFT_HEIGHT,2,TFT_ORANGE,TFT_RED);

  //tft.init();
  //tft.setRotation(3);
  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_DARKGREY);

  //tft.setTextSize(1);
  //tft.setTextFont(1);
  //tft.println("V");
  //tft.color565(ir, ig, ib)

  //tft.fillRect(tft.cursor_x + x*scale, tft.cursor_y + y*scale, scale, scale, Color);
  //tft.drawFastHLine(centerX - halfCrossSize, centerY, 2 * halfCrossSize + 1, TFT_WHITE);
  //tft.drawFastVLine(centerX, centerY - halfCrossSize, 2 * halfCrossSize + 1, TFT_WHITE);
  //tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_BLACK);

  tft.setTextSize(2);
  tft.setCursor(0, tft.height() / 2);
  tft.print("WORKING...");
}

void setup(void) {  
  Serial.begin(115200);
  setup_wifi();
  setup_ArduinoOTA();
  setup_TFT();
  
  btn1.begin(BUTTON_1);
  //btn1.setTapHandler(event_on_btn1);
  btn1.setClickHandler(event_on_btn1);

  ////Init DS18B20 Sensor
  //sensors.begin();
  //sensors.requestTemperatures();
  //sensors.setWaitForConversion(false); //switch to async mode
  //CHCurTemp, pv_last = sensors.getTempCByIndex(0);
  lastOTSetTempMillis = millis();

  //Init OpenTherm Controller
  OpenThermIf.begin(handleInterrupt);

  //Init MQTT Client
  MQTTClient.setServer(mqtt_server, mqtt_port);
  MQTTClient.setCallback(MQTTcallback);
}

void loop(void) { 
  ArduinoOTA.handle();

  curMillis = millis();
  if (curMillis - lastOTSetTempMillis > 1000) {   
    //Set/Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;
    unsigned long response = OpenThermIf.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    OpenThermResponseStatus responseStatus = OpenThermIf.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      String str="Error: Invalid boiler response ";
      str = str + String(response, HEX);
      Serial.println(str);
      MQTTClient.publish("/home1/OpenTherm/ERR", str.c_str());
    }   
    
    /*CHCurTemp = sensors.getTempCByIndex(0);
    dt = (new_ts - lastOTSetTempMillis) / 1000.0;
    lastOTSetTempMillis = new_ts;
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      op = pid(CHSetPoint, CHCurTemp, pv_last, ierr, dt);
      //Set Boiler Temperature
      OpenThermIf.setBoilerTemperature(op);
    }
    pv_last = CHCurTemp;
    
    sensors.requestTemperatures(); //async temperature request
    */

    lastOTSetTempMillis = curMillis;
    OpenThermIf.setBoilerTemperature(CHSetPoint);
    MQTTpublish_temperature();
  }
  
  //MQTT Loop
  if (!MQTTClient.connected()) {
    MQTTReconnect();
  }
  MQTTClient.loop();
  
  btn1.loop();
}