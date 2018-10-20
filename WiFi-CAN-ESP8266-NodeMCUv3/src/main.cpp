/* Comment this out to disable prints and save space */
//#define BLYNK_PRINT Serial
//#define testmode

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <NIK_ONLY_WifiBlynk.h>

#include <mcp_can.h>
#include <SPI.h>
#include <QueueList.h>

// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT to pin 2 (D4 on NodeMCU)
MCP_CAN CAN0(4);                               // Set CS to pin 4 (D2 on NodeMCU)

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];
//filter message types:
#define CAN_MSG_MASK           0xF0L
#define CAN_MSG_FILTER_UNITCMD 0x80L  //command for special unit
#define CAN_MSG_FILTER_UNITINF 0x40L  //some info for special unit
#define CAN_MSG_FILTER_INF     0x20L  //some info for everyone who wants it (e.g. outer temperature)
#define CAN_MSG_FILTER_STATIST 0x10L  //non-critical statistics
//receive only messages to this unit - include recever id:
#define CAN_Unit_MASK         0x0FL
#define CAN_Unit_FILTER_KUHFL 0x01L //Floor temperature tegulation unit
#define CAN_Unit_FILTER_ESPWF 0x02L //ESP8266 WiFi-CAN bridge
#define CAN_Unit_FILTER_ELCT1 0x03L //Electric power control
#define CAN_Unit_FILTER_OUTDT 0x04L //Unit for outdoor temperature

struct CANMessage{ unsigned char vPinNumber; float vPinValueFloat; byte nTries; };
int timerIntervalForNextSendCAN=0;
int CANQueueError=0; 
#define CANQueueErrorOverflow 7777
static int CANQueueMaxLength=30;
QueueList <CANMessage> CANQueue;

BlynkTimer timer;
////////////////////////////////////////////////////////////////
void addCANMessage2Queue(unsigned char vPinNumber, float vPinValueFloat);

#ifdef testmode
void testSendQueue(){
  for(int i=0;i<10;i++){
    addCANMessage2Queue(20+i,i*100);
  }
}
#endif

/////////////////////////////////////////////////////////////////
  
char sendVPinCAN(unsigned char vPinNumber, float vPinValueFloat){ //transfer by CAN
  
  *rxBuf = vPinNumber;
	*(float*)(rxBuf+1) = vPinValueFloat;

	// send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
	byte sndStat = CAN0.sendMsgBuf((0x100L | CAN_MSG_FILTER_STATIST | CAN_Unit_FILTER_OUTDT)<<16, 0, 1+sizeof(float), rxBuf);

  //data[0] = vPinNumber;
  //*(float*)(data+1) = fvalue;
  //byte sndStat = CAN0.sendMsgBuf(0x100, 1, 1 + sizeof(float), data);

	if(sndStat == CAN_OK){
		//Serial.println("Message Sent Successfully!");
    return sndStat;
	} else {
		Serial.print("Error (");
    Serial.print(sndStat);
    Serial.println(") Sending Message by CAN bus!..");
    return sndStat;
	}
}

//SENDNEXT ////////////////////////////////////////////////////////////////////////////
void sendNextCANMessage(){ 

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
    return; //nothing to send is also good
  }

  //queue is not empty:
  CANMessage mes = CANQueue.peek();
  
  #ifdef testmode
  Serial.print("   queue: sending VPIN=");
  Serial.print(mes.vPinNumber);
  Serial.print(" Value=");
  Serial.print(mes.vPinValueFloat);
  #endif

  char res = sendVPinCAN( mes.vPinNumber, mes.vPinValueFloat );

  if(res == CAN_OK){
    CANQueue.pop(); //drop this message
  }else{
    mes.nTries++;
    if( mes.nTries > 20 ){
      CANQueue.pop(); //drop this message
      CANQueueError = res;
    }
  }
  #ifdef testmode
  Serial.print(" res=");
  Serial.println(res);
  #endif

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
  }else{ //not empty - try again soon:
    timerIntervalForNextSendCAN = timer.setTimeout( 2, sendNextCANMessage ); //2 millis try interval
  }
}

//ADD /////////////////////////////////////////////////////////////////////////////////
void addCANMessage2Queue(unsigned char vPinNumber, float vPinValueFloat){
  if(CANQueue.count()>=CANQueueMaxLength){
    CANQueueError = CANQueueErrorOverflow;
    Serial.println("Can't add to queue: CAN queue overflow!");
    return;
  }
  CANQueue.push( CANMessage{ vPinNumber, vPinValueFloat, 0} );
  if(timerIntervalForNextSendCAN==0){
    timerIntervalForNextSendCAN = timer.setTimeout( 2, sendNextCANMessage); //2 millis try interval
  }
}
///////////////////////////////////////////////////
void checkReadCAN(){
  if(!digitalRead(CAN0_INT))                          // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)

    Blynk.virtualWrite(rxBuf[0], *(float*)(rxBuf+1));

    #ifdef testmode
    //sprintf seems not to work on ESP-Arduino:
    // if((rxId & 0x80000000) == 0x80000000)             // Determine if ID is standard (11 bits) or extended (29 bits)
    //   sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    // else
    //   sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    //Serial.print(msgString);
    Serial.print( "Received message: Standard ID = ");
    Serial.print(((rxId & 0x80000000) == 0x80000000)?"Extended ID: ":"Standard ID:");
    Serial.print(rxId);
    Serial.print(" f=");
    Serial.print(*(float*)(rxBuf+1));
    Serial.println();

    // if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
    //   Serial.print(" REMOTE REQUEST FRAME");
    // } else {
    for(byte i = 0; i<len; i++){
        //sprintf(msgString, " 0x%.2X", rxBuf[i]);
        //Serial.print(msgString);
        Serial.print(" ");
        Serial.print(rxBuf[i]);
    }
    //}
    Serial.println();
    #endif
  }
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
    addCANMessage2Queue(request.pin, i.asFloat());/////////////////send to CAN bus
    
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
//BLYNK_CONNECTED() {
//  //get data stored in virtual pin V0 from server
//  Blynk.syncVirtual(V0, V2);
//}

void setup(){
  // Debug console
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY
    ;//Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515!..");
  
  //initialize filters Masks(0-1),Filters(0-5):
  //unsigned long mask = (0x0100L | CAN_Unit_MASK)<<16;			//0x0F	0x010F0000;
  //unsigned long filt = (0x0100L | CAN_Unit_FILTER_ESPWF)<<16;	//0x04	0x01040000;
  //CAN0.init_Mask(0,0,mask);                // Init first mask...
  //CAN0.init_Filt(0,0,filt);                // Init first filter...
  //CAN0.init_Filt(1,0,filt);                // Init second filter...
  //CAN0.init_Mask(1,0,mask);                // Init second mask...
  //CAN0.init_Filt(2,0,filt);                // Init third filter...
  //CAN0.init_Filt(3,0,filt);                // Init fouth filter...
  //CAN0.init_Filt(4,0,filt);                // Init fifth filter...
  //CAN0.init_Filt(5,0,filt);                // Init sixth filter...

  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);
  #ifdef testmode
  CAN0.setMode(MCP_LOOPBACK);
  #endif
  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input

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
