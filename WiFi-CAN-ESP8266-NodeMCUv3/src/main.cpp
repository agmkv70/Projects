/* Comment this out to disable prints and save space */
//#define BLYNK_PRINT Serial

//#define testmode
//#define testmodeCAN
//#define MQTT_On
#define LocalBlynk_On

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <NIK_ONLY_WifiBlynk.h>

#define CAN_PIN_INT 2   // Set INT to pin 2 (D4 on NodeMCU)
#define CAN_PIN_CS 4    // Set CS to pin 4 (D2 on NodeMCU)

#define _MAX_FIXEDARRAY_DEFINED 50 //default=20
#include <NIK_defs.h>
#include <NIK_can.h>

#ifdef MQTT_On
////////////////////////////////////////////////////////////////
#include <PubSubClient.h>
#define BUFFER_SIZE 200

WiFiClient espClient;
PubSubClient MQTTClient(espClient);
long lastReconnectMQTTmillis=0;
////////////////////////////////////////////////////////////////

int mqttmesonoff=0;
//#ifdef testmode
void testSendQueueMQTT(){
  #ifdef testmode
  Serial.println("test:sending mqtt...");
  #endif
  mqttmesonoff++;
  mqttmesonoff%=10;
  for(int i=1;i<=1;i++){
    //addCANMessage2Queue(0,70+i,i);

    String topicPinName = "/home1/VPIN_";
    topicPinName += (int)(i+70);
    char val[20];
    itoa(mqttmesonoff,val,10);
    MQTTClient.publish(topicPinName.c_str(), val);		////////// send to MQTT broker topic="71" - i++ every 10 sec. ?!
  }
  //#ifdef testmode
  //Blynk.syncVirtual(VPIN_Home);
  //Blynk.syncVirtual(VPIN_OutdoorTemp);
  //Blynk.syncVirtual(VPIN_BoilerPower);
  //setReceivedVirtualPinValue(69,100);
  //#endif
}
//#endif
#endif

void SendCANQueueError(){
  Blynk.virtualWrite(VPIN_CANQueueError, CANQueueError);
}

/////////////////////////////////////////////////////////////////
  
char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
  Blynk.virtualWrite(vPinNumber, vPinValueFloat);

  #ifdef MQTT_On
  String topicPinName = "/home1/VPIN_";
  topicPinName += (int)vPinNumber;
  char strval[20];
  dtostrf(vPinValueFloat,10,1,strval);
  MQTTClient.publish(topicPinName.c_str(), strval);		////////// send to MQTT broker
  #endif

  return 0;
}

// Receive data from App and send to CAN bus and MQTT broker:
// This is called for all virtual pins, that don't have BLYNK_WRITE handler
BLYNK_WRITE_DEFAULT() {
  #ifdef testmode
  Serial.print("input V");
  Serial.print(request.pin);
  Serial.println(":");
  #endif

  // Print all parameter values
  for (auto i = param.begin(); i < param.end(); ++i) {
    addCANMessage2Queue(0,request.pin, i.asFloat());/////////////////send to CAN bus
    
    #ifdef testmode
    Serial.print("* ");
    Serial.println(i.asString());
    #endif

    /*
    String topicPinName = "/home1/VPIN_set_";
    topicPinName += (int)request.pin;
    MQTTClient.publish(topicPinName.c_str(), i.asString());		////////// send to MQTT broker
    */
  }
}

//when from CANbus to App??
//BLYNK_READ_DEFAULT() { // This is called for all virtual pins, that don't have BLYNK_READ handler
//   // Generate random response
//   int val = random(0, 100);
//   Serial.print("output V");
//   Serial.print(request.pin);
//   Serial.print(": ");
//   Serial.println(val);
//   Blynk.virtualWrite(request.pin, val);
// }

// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
  //get data stored in virtual pin V0 from server. Server will push these pins and BLYNK_WRITE will be 
  Blynk.syncVirtual(VPIN_BoilerTargetTemp,
                    VPIN_STATUS,
                    VPIN_PIDSTATUS,
                    VPIN_VALVESTATUS,
                    VPIN_ManualFloorIn,
                    VPIN_tempTargetFloorOut,
                    VPIN_SetMainCycleInterval,
                    VPIN_SetBoilerPowerPeriodMinutes,
                    VPIN_BoilerPID_Kp,
                    VPIN_BoilerPID_Ki,
                    VPIN_BoilerPID_Kd,
                    VPIN_HomePID_Kp,
                    VPIN_HomePID_Ki,
                    VPIN_HomePID_Kd,
                    VPIN_HomeTargetTemp,
                    VPIN_LEDSetPWMch1,
                    VPIN_LEDSetPWMch2,
                    VPIN_LEDSetPWMch3,
                    VPIN_LEDSetPWMch4
                   );
  //if(VPIN_PIDSTATUS!=2){ //2==(Home,autoBoilTrg)
  //  Blynk.syncVirtual(VPIN_BoilerTargetTemp); //because in this case it is evaluated automatically and we mustn't bother it on conn/disconn
  //}
}

#ifdef MQTT_On
void MQTTReconnect() {
  long curMillis = millis();
  if( curMillis - lastReconnectMQTTmillis < 5000 ){ 
    return; //try reconnect not too often - once in 5 sec.
  }
  lastReconnectMQTTmillis = curMillis;

  Serial.print("Attempting MQTT connection... ");
  
  #ifdef testmode
  String clientId = "ESP_CAN_bridge_test";
  #endif
  #ifndef testmode
  String clientId = "ESP_CAN_bridge2";
  #endif
  
  //clientId += String(random(0xffff), HEX);
  
  // Attempt to connect
  if( MQTTClient.connect(clientId.c_str(), mqtt_user, mqtt_pass) ) {
    
    #ifdef testmode
    Serial.println("MQTT:connected");
    #endif
    
    //#ifdef testmode
    MQTTClient.publish("HelloOnConnectTopic", "hello, its ESP in testMode!");
    //#endif
    // ... and resubscribe
    MQTTClient.subscribe("/home/VPIN_Command/#");

  }else{
    #ifdef testmode
    Serial.print("MQTT: connection failed, rc=");
    Serial.print(MQTTClient.state());
    Serial.println(" try again in 5 seconds");
    #endif
  }
  
}

void MQTTCallback(char* topic, byte* payload, unsigned int length){ //receive from mqtt brocker incoming mes
  #ifdef testmode
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
  
  if(!strncmp(topic,"/home/VPIN_Command/",19)) // Проверяем из нужного ли нам топика пришли данные
  { int topicPin = String((char*)(topic+19)).toInt();
    float fvalue = atof((char*)payload);
  
    #ifdef testmode
    Serial.print("MQTTCallback: topicPIN= ");
    Serial.print(topicPin);
    Serial.print("floatValue= ");
    Serial.println(fvalue);
    #endif

    addCANMessage2Queue(0, topicPin, fvalue);//////////resend to CAN bus
    
  }else{
    #ifdef testmode
    Serial.print("MQTTCallback: unknown topic = ");
    Serial.println(topic);
    #endif
  }
}
#endif

#ifdef WifiLED_On
#define LPin D8
void yLEDBlink();
#endif

void setup(){
  #ifdef testmode
    Serial.begin(115200);
  #endif

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STD, CAN_250KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY,MCP_STDEXT
    ;//Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515!..");
  
  //initialize filters Masks(0-1),Filters(0-5):
  unsigned long mask = (0x0100L | CAN_Unit_MASK)<<16;			//0x0F	0x010F0000;
  unsigned long filt = (0x0100L | CAN_Unit_FILTER_ESPWF)<<16;	//0x04	0x01040000;
  //first mask: ID=0x100
  CAN0.init_Mask(0,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
  CAN0.init_Filt(1,0,0x01000000);                // Init second filter...
  //second mask: ID=0x010F - receive only CAN_Unit_MASK = CAN_Unit_FILTER_ESPWF
  CAN0.init_Mask(1,0,mask);                // Init second mask...
  CAN0.init_Filt(2,0,filt);                // Init third filter...
  CAN0.init_Filt(3,0,filt);                // Init fouth filter...
  CAN0.init_Filt(4,0,filt);                // Init fifth filter...
  CAN0.init_Filt(5,0,filt);                // Init sixth filter...

  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);
  #ifdef testmode
    CAN0.setMode(MCP_LOOPBACK);
  #endif
  pinMode(CAN_PIN_INT, INPUT);                           // Configuring pin for /INT input

  //Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  #ifndef LocalBlynk_On
  Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8442);
  #endif
  #ifdef LocalBlynk_On
  Blynk.begin(auth, ssid, pass, IPAddress(192,168,0,130), 8080);
  #endif

  timer.setInterval(5000L, SendCANQueueError);

  #ifdef MQTT_On
  MQTTClient.setServer(mqtt_server, mqtt_port);
  MQTTClient.setCallback(MQTTCallback);
  
  //timer.setInterval(1L, checkReadCAN);
  #ifdef testmode
    timer.setInterval(10000L, testSendQueue);
  #endif
  //#ifndef testmode
  //  timer.setInterval(60000L, testSendQueue);
  //#endif
  #endif

  #ifdef WifiLED_On
  timer.setInterval(300,yLEDBlink);
  pinMode(LPin,OUTPUT);
  //digitalWrite(LPin,HIGH);
  //delay(500);
  //digitalWrite(LPin,LOW);
  //delay(500);
  #endif
}

#ifdef WifiLED_On
int yLED=1,yblink=0;
void yLEDBlink(){
  if(yLED==0){
    digitalWrite(LPin,LOW);
  }else if(yLED==1){
    digitalWrite(LPin,HIGH);
  }else{ //2==blink
    if(yblink==0){
      digitalWrite(LPin,HIGH);
      yblink=1;
    }else{
      yblink=0;
      digitalWrite(LPin,LOW);
    } 

  }
}
#endif

void loop(){
  Blynk.run();
  timer.run();
  checkReadCAN();

  #ifdef WifiLED_On
  if (WiFi.status() == WL_CONNECTED){
    yLED=1;
    
    #ifdef MQTT_On
    if (MQTTClient.connected())
      MQTTClient.loop();
    else
      MQTTReconnect();
    #endif
  }else
    yLED=2;
  #endif
}
