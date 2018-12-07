/* Comment this out to disable prints and save space */
//#define BLYNK_PRINT Serial

//#define testmode

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <NIK_ONLY_WifiBlynk.h>

#define CAN_PIN_INT 2   // Set INT to pin 2 (D4 on NodeMCU)
#define CAN_PIN_CS 4    // Set CS to pin 4 (D2 on NodeMCU)

#include <NIK_defs.h>
#include <NIK_can.h>

////////////////////////////////////////////////////////////////

#ifdef testmode
void testSendQueue(){
  for(int i=0;i<10;i++){
    addCANMessage2Queue(0,20+i,i*100);
  }
}
#endif

/////////////////////////////////////////////////////////////////
  
char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
  Blynk.virtualWrite(vPinNumber, vPinValueFloat);
  return 0;
}

// Receive data from App to CANbus
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
  Blynk.syncVirtual(VPIN_STATUS,
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
  if(VPIN_PIDSTATUS!=2){ //2==(Home,autoBoilTrg)
    Blynk.syncVirtual(VPIN_BoilerTargetTemp); //because in this case it is evaluated automatically and we mustn't bother it on conn/disconn
  }
}

void setup(){
  // Debug console
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY,MCP_STDEXT
    ;//Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515!..");
  
  //initialize filters Masks(0-1),Filters(0-5):
  unsigned long mask = (0x0100L | CAN_Unit_MASK)<<16;			//0x0F	0x010F0000;
  unsigned long filt = (0x0100L | CAN_Unit_FILTER_ESPWF)<<16;	//0x04	0x01040000;
  CAN0.init_Mask(0,0,0x01FFFFFF);                // Init first mask...
  CAN0.init_Filt(0,0,0);                // Init first filter...
  CAN0.init_Filt(1,0,0);                // Init second filter...
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
  Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8442);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8442);

  //timer.setInterval(1L, checkReadCAN);
  #ifdef testmode
  timer.setInterval(4000L, testSendQueue);
  #endif
}

void loop(){
  Blynk.run();
  timer.run();
  checkReadCAN();
}
