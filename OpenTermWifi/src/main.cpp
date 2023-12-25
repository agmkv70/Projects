#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//#include <ESP8266WiFi.h> //gone to esp32
#include <PubSubClient.h>
#include "NIK_ONLY_WifiBlynk.h" //wifi and mqtt credentials
#include <OpenTherm.h>

/////////////////////TTGO start:
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
/////////////////////:TTGO end

#include <NTPtime.h>
NTPtime Time(2); //UA in +2 time zone
DSTime dst(3, 0, 7, 3, 10, 0, 7, 4); //https://en.wikipedia.org/wiki/Eastern_European_Summer_Time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

#include <EEPROM.h>
template <class T> int EEPROM_WriteAnything_seq(int &ee, const T& value) //write any type to address ee and return numbytes
{
   const byte* p = (const byte*)(const void*)&value;
   unsigned int i;
   for (i = 0; i < sizeof(value); i++)
   {   //EEPROM.update(ee++, *p++);
      byte val = EEPROM.read(ee);
      if(val!=*p)
        EEPROM.write(ee,*p);
      ee++;
      p++;
   }
   return i;
}
template <class T> int EEPROM_ReadAnything_seq(int &ee, T& value) //read any type from address ee and return numbytes
{
   byte* p = (byte*)(void*)&value;
   unsigned int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return i;
}
class EEPROMStorageClass{
  public:
  byte CHEnable;
  byte DHWEnable;
  float CHSetPoint;
  float DHWt;
  byte CHMode;
  
  EEPROMStorageClass(){
    CHEnable=0;
    DHWEnable=0;
    CHSetPoint=24;
    DHWt=53;
    CHMode=0;
  }
  int uid=1; //unique key to sign 
  int _offset;
  void storeAll(){
    _offset=0;
    EEPROM_WriteAnything_seq(_offset,uid);
    EEPROM_WriteAnything_seq(_offset,CHEnable);
    EEPROM_WriteAnything_seq(_offset,DHWEnable);
    EEPROM_WriteAnything_seq(_offset,CHSetPoint);
    EEPROM_WriteAnything_seq(_offset,DHWt);
    EEPROM_WriteAnything_seq(_offset,CHMode);
  };
  void restoreAll(){
    int read_uid;
    EEPROM_ReadAnything_seq(_offset,read_uid);
    if(read_uid != uid){ //never wrote valid values into eeprom
      storeAll(); //write defaults
      return; 
    }
    EEPROM_ReadAnything_seq(_offset,CHEnable);
    EEPROM_ReadAnything_seq(_offset,DHWEnable);
    EEPROM_ReadAnything_seq(_offset,CHSetPoint);
    EEPROM_ReadAnything_seq(_offset,DHWt);
    EEPROM_ReadAnything_seq(_offset,CHMode);
  };
};
EEPROMStorageClass EEPROMStorage;
float CHCurTemp = 0; //current temperature
float curDHWFlowRate = 0;

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 12;
const int outPin = 13;
OpenTherm OpenThermIf(inPin, outPin);

WiFiClient espClient;
PubSubClient MQTTClient(espClient);
char buf[10];

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
  Serial.print("WiFi connecting to ");
  Serial.print(ssid);
  Serial.print(" .... ");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(10000);
    ESP.restart();
  }

  Serial.println("OK: connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

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

void MQTTReconnect() {  
  long curMillis = millis();
  if( curMillis - lastReconnectMQTTmillis < 5000 ){ 
    return; //try reconnect not too often - once in 5 sec.
  }
  lastReconnectMQTTmillis = curMillis;

  while (!MQTTClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTTClient.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //MQTTpublish_CurTemp();
      // ... and resubscribe
      MQTTClient.subscribe("/home1/OpenTherm/Set/#");
      MQTTClient.subscribe("/home1/OpenTherm/Read/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }    
  }
}

void MQTTcallback(char* topic, byte* payload, uint length){
  Serial.print("MQTT_IN:: ");
  Serial.println(topic);
  Serial.println(strcmp(topic, "/home1/OpenTherm/Set/CHSetPoint"));
  
  String str = String();    
  for (uint i = 0; i < length; i++){
    str += (char)payload[i];
  }
  
  if(strcmp(topic, "/home1/OpenTherm/Set/CHSetPoint") == 0){
    EEPROMStorage.CHSetPoint = str.toFloat();
    Serial.print(" CHSetPoint = ");  
    Serial.println(EEPROMStorage.CHSetPoint);

  }else if(strcmp(topic, "/home1/OpenTherm/Set/CHEnable")== 0){
    EEPROMStorage.CHEnable = (byte)str.toInt();
    Serial.print(" CHEnable = ");  
    Serial.println(EEPROMStorage.CHEnable);

  }else if(strcmp(topic, "/home1/OpenTherm/Set/DHWt") == 0){
    EEPROMStorage.DHWt = str.toFloat();
    Serial.print(" DHWt = ");  
    Serial.println(EEPROMStorage.DHWt);

  }else if(strcmp(topic, "/home1/OpenTherm/Set/DHWEnable")== 0){
    EEPROMStorage.DHWEnable = (byte)str.toInt();
    Serial.print(" DHWEnable = ");  
    Serial.println(EEPROMStorage.DHWEnable);

  }else if(strstr(topic, "/home1/OpenTherm/Read/HEX/") != NULL){
    char strid[4];
    uint i=0;
    for(; i<3 && topic[26+i]!=0; i++){
      strid[i]=topic[26+i];
      strid[i+1]=0;
    }

    Serial.print(" Read/HEX/");  
    Serial.print(atoi(strid));
    Serial.print(" data=");
    Serial.print(str);
    
    uint res = OpenThermIf.getAny(atoi(strid),strtol(str.c_str(), 0, 16));

    char ret_topic[101];
    for(i=0; i<100 && topic[i]!=0; i++){
      ret_topic[i]=topic[i];
      ret_topic[i+1]=0;
    }
    ret_topic[20]='R';
    
    OpenThermResponseStatus responseStatus = OpenThermIf.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS){
      Serial.print(" result= ");
      Serial.print("0x");
      Serial.print(res < 16 ? "0" : "");
      Serial.println(res, HEX);
      
      char myHex[10] = "";
      ltoa(res,myHex,16);
      MQTTClient.publish(ret_topic, myHex);

    }else{ //error
      Serial.print(" ERR= ");
      Serial.println(responseStatus);

      switch(responseStatus){
        case OpenThermResponseStatus::NONE :
          MQTTClient.publish(ret_topic, "ERROR_NONE");
        break;
        case OpenThermResponseStatus::INVALID :
          MQTTClient.publish(ret_topic, "ERROR_INVALID");
        break;
        case OpenThermResponseStatus::TIMEOUT :
          MQTTClient.publish(ret_topic, "ERROR_TIMEOUT");
        break;
        default:
          MQTTClient.publish(ret_topic, "ERROR_?");
      }
      
    }
  }else{
    Serial.println("ERR: unknown MQTT topic!");  
    return; //jump off without val storage
  }

  EEPROMStorage.storeAll();
}

void MQTTpublish_CHCurTemp() {
  String str_CHCurTemp=String(CHCurTemp);
  Serial.print("MQTT OUT:: curt=");
  Serial.println(str_CHCurTemp);    
  //String(CHCurTemp).toCharArray(buf, 10);
  MQTTClient.publish("/home1/OpenTherm/CHCurTemp", str_CHCurTemp.c_str());  
}

void MQTTpublish_DHWFlowRate() {
  String str=String(curDHWFlowRate);
  Serial.print("MQTT OUT:: DHWFlowRate=");
  Serial.println(str);    
  MQTTClient.publish("/home1/OpenTherm/DHWFlowRate", str.c_str());  
}

void setup(void) {
  Serial.begin(115200);

  EEPROMStorage.restoreAll();
  setup_wifi();
  setup_ArduinoOTA();
  setup_TFT();
  
  Time.setDSTauto(&dst);
  Time.begin();
  //Time.tick(); //works only in loop
  //Serial.println("Local time: ");
  //Serial.println(Time.timeString());
  //Serial.println(Time.dateString());
  //Serial.println();

  btn1.begin(BUTTON_1);
  //btn1.setTapHandler(event_on_btn1);
  btn1.setClickHandler(event_on_btn1);

  lastOTSetTempMillis = millis();

  //Init OpenTherm Controller
  OpenThermIf.begin(handleInterrupt);

  //Init MQTT Client
  MQTTClient.setServer(mqtt_server, mqtt_port);
  MQTTClient.setCallback(MQTTcallback);
}

void loop(void) { 
  ArduinoOTA.handle();
  Time.tick();
  //if (Time.ms() == 0) {// секунда почалась / second started

  curMillis = millis();
  if (curMillis - lastOTSetTempMillis > 1000) {   
    lastOTSetTempMillis = curMillis;
    
    //Set/Get Boiler Status
    unsigned long response = OpenThermIf.setBoilerStatus(EEPROMStorage.CHEnable, EEPROMStorage.DHWEnable);
    OpenThermResponseStatus responseStatus = OpenThermIf.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      String str="OT:Error getting status: Invalid boiler response ";
      str = str + String(response, HEX);
      Serial.println(str);
      MQTTClient.publish("/home1/OpenTherm/ERR", str.c_str());

    }else{ //SUCCESS
      OpenThermIf.setBoilerTemperature(EEPROMStorage.CHSetPoint);
      
      OpenThermIf.setDHWSetpoint(EEPROMStorage.CHEnable==0 ? 0 : EEPROMStorage.DHWt);

      CHCurTemp = OpenThermIf.getBoilerTemperature();
      //responseStatus = OpenThermIf.getLastResponseStatus();
      //if (responseStatus != OpenThermResponseStatus::SUCCESS)
      MQTTpublish_CHCurTemp();

      curDHWFlowRate = OpenThermIf.getDHWFlowRate();
      MQTTpublish_DHWFlowRate();

    }
    
    //Serial.println("Local time: ");
    //Serial.println(Time.timeString()); 	//виводимо / outputting
    //Serial.println(Time.dateString());
    //Serial.println();
  }
  
  //MQTT Loop
  if (!MQTTClient.connected()) {
    MQTTReconnect();
  }
  MQTTClient.loop();
  
  btn1.loop();
}