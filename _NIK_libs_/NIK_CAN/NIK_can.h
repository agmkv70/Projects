//NIK_can.h
//all for can

#include <SimpleTimer.h>
#include <mcp_can.h>
#include <SPI.h>
#include <QueueListFIXEDARRAY.h>
#include <EEPROM.h>

//#define testmode
//#define testmodeCAN

//#define CAN_PIN_INT  9        // INT = pin 9
MCP_CAN CAN0(CAN_PIN_CS);       // CS  = pin 10

unsigned long rxId;
unsigned char dataLen = 0;
unsigned char rxBuf[8];
#define CAN_NEXT_TRY_INTERVAL 50
#define CAN_MAX_SEND_TRIES 5

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

SimpleTimer timer;

struct CANMessage{ long mesID; unsigned char vPinNumber; byte isStr; float vPinValueFloat; String Str; byte nTries; };
int timerIntervalForNextSendCAN=0;
int CANQueueError=0; 
int CANQueueStop=0; //try stopping while adding values in series, then run sending
#define CANQueueErrorOverflow 7777
static int CANQueueMaxLength=_MAX_FIXEDARRAY_DEFINED;//default 20, 50 for wifi
QueueListFA <CANMessage> CANQueue;

//only for ESP8166:
//extern "C" {
//  #include "user_interface.h"
//}

///////////////////////////////////////CAN//////////////////////////////////////////
char sendVPinCAN(long mesID, unsigned char vPinNumber, byte isStr, float vPinValueFloat, String Str){ //transfer by CAN
  byte sndStat=CAN_OK;

	*rxBuf = vPinNumber;
  if(isStr==0){
    *(float*)(rxBuf+1) = vPinValueFloat;

    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat = CAN0.sendMsgBuf((0x100L | mesID), 0, 1+sizeof(float), rxBuf); //<<16

  }else{ //filling buffer with string and sending(1-7 bytes):
    byte i=0;
    for(; i<Str.length() && i<8; i++)
      *(char*)(rxBuf+1+i) = Str[i];

    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat = CAN0.sendMsgBuf((0x100L | mesID), 0, 1+i, rxBuf); //<<16
  }

	if(sndStat == CAN_OK){
		//Serial.println("Message Sent Successfully!");
    return sndStat;
	} else {
    #ifdef testmode
		Serial.print("Error (");
    Serial.print(sndStat);
    Serial.println(") Sending Message by CAN bus!..");
    #endif
    return sndStat;
	}
}

//SENDNEXT ////////////////////////////////////////////////////////////////////////////
void sendNextCANMessage(){ 

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
    return; //nothing to send is also good
  }
  if(CANQueueStop){
    timerIntervalForNextSendCAN = timer.setTimeout( CAN_NEXT_TRY_INTERVAL, sendNextCANMessage ); //5 millis try interval
    return;
  }

  //queue is not empty:
  CANMessage *mes = CANQueue.peek();
  
  #ifdef testmodeCAN
  Serial.print("  >Sending queue#");
  Serial.print(CANQueue.count());
  Serial.print(" :id=");
  Serial.print(mes->mesID);
  Serial.print(" VPIN=");
  Serial.print(mes->vPinNumber);
  Serial.print(" Value=");
  Serial.print(mes->vPinValueFloat);
  Serial.print(" try=");
  Serial.print(mes->nTries);
  //only for ESP8166:
  //Serial.print(" leftMEM=");
  //Serial.print(system_get_free_heap_size());
  #endif

  char res = sendVPinCAN( mes->mesID, mes->vPinNumber, mes->isStr, mes->vPinValueFloat, mes->Str);

  if(res == CAN_OK){
    CANQueue.drop(); //drop this message
    //free(mes);
  }else{ //sending error:
    mes->nTries++;
    if( mes->nTries > CAN_MAX_SEND_TRIES ){
      CANQueue.drop(); //drop this message
      //free(mes);
      CANQueueError = res;
    }
  }
  #ifdef testmodeCAN
  Serial.print(" res=");
  Serial.println(res);
  #endif

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
  }else{ //not empty - try again soon:
    timerIntervalForNextSendCAN = timer.setTimeout( CAN_NEXT_TRY_INTERVAL, sendNextCANMessage ); //5 millis try interval
  }
}

//ADD /////////////////////////////////////////////////////////////////////////////////
//addCANMessage2Queue(CAN_MSG_FILTER_STATIST | CAN_Unit_FILTER_ESPWF, VPIN_OutdoorTemp, tempOutdoor);
void addCANMessage2Queue(long mesID, unsigned char vPinNumber, float vPinValueFloat){  //older version - only float! 
  if(CANQueue.count()>=CANQueueMaxLength){
    CANQueueError = CANQueueErrorOverflow;
    #ifdef testmode
    Serial.print("Can't add to queue: CAN queue overflow! on vPin=");
    Serial.print(vPinNumber);
    #endif
    return;
  }
  CANQueueStop=1;
  #ifdef testmodeCAN
  Serial.print("   pushing: id=");
  Serial.print(mesID);
  Serial.print(" VPIN=");
  Serial.print(vPinNumber);
  Serial.print(" Value=");
  Serial.print(vPinValueFloat);
  #endif
  CANMessage mes=CANMessage{mesID, vPinNumber, 0, vPinValueFloat, "", 0};
  CANQueue.push(&mes);
  if(timerIntervalForNextSendCAN==0){
    timerIntervalForNextSendCAN = timer.setTimeout( CAN_NEXT_TRY_INTERVAL, sendNextCANMessage); //5 millis try interval
  }
  CANQueueStop=0;
}

void addCANMessage2QueueStr(long mesID, unsigned char vPinNumber, String Str){  //new version with char* and flag!
  if(CANQueue.count()>=CANQueueMaxLength){
    CANQueueError = CANQueueErrorOverflow;
    #ifdef testmode
    Serial.print("Can't add to queue(str): CAN queue overflow! on vPin=");
    Serial.print(vPinNumber);
    #endif
    return;
  }
  CANQueueStop=1;
  #ifdef testmodeCAN
  Serial.print("   pushing: id=");
  Serial.print(mesID);
  Serial.print(" VPIN=");
  Serial.print(vPinNumber);
  Serial.print(" Value=");
  Serial.print(vPinValueFloat);
  #endif
  CANMessage mes=CANMessage{mesID, vPinNumber, 1, 0, Str, 0};
  CANQueue.push(&mes);
  if(timerIntervalForNextSendCAN==0){
    timerIntervalForNextSendCAN = timer.setTimeout( CAN_NEXT_TRY_INTERVAL, sendNextCANMessage); //5 millis try interval
  }
  CANQueueStop=0;
}

//////////////////////////////////////////////////////////////////MAIN cycle:
void MainCycle_StartEvent();
// {
    //   switch (STATUS)
    //   {
    // 		case Status_Standby:
    // 			return;	//skip standby state
    // 	}
    //
    // 	TempDS_AllStartConvertion();
    //     #ifdef testmode
    // 	Serial.println("Start coversion... ");
    //     #endif
    //
    // 	timer.setTimeout(1000L, MainCycle_ReadTempEvent); //start once after timeout
// }

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat);

void checkReadCAN() {
  if(digitalRead(CAN_PIN_INT)==HIGH){ //HIGH = no CAN received messages in buffer
	  return;
  }
  //If CAN0_INT pin is LOW, read receive buffer:
  CAN0.readMsgBuf(&rxId, &dataLen, rxBuf);      // Read data: len = data length, buf = data byte(s)

  #ifdef testmodeCAN
  //print received message to Serial:
  Serial.print( "Received message: ");
  Serial.print(((rxId & 0x80000000) == 0x80000000)?"Extended ID: ":"Standard ID:");
  Serial.print(rxId);
  Serial.print(" f=");
  Serial.print(*(float*)(rxBuf+1));
  Serial.println();
  // if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
  //   Serial.print(" REMOTE REQUEST FRAME");
  // } else {
  for(byte i = 0; i<dataLen; i++){
      Serial.print(" ");
      Serial.print(rxBuf[i]);
  }
  //}
  Serial.println();
  #endif

  if(dataLen<2)
    return; //we need at least 5 bytes (1 = number of VPIN, 2-5 = float value (4 bytes))
  //vPinNumber = *rxBuf; //first byte is Number of Virtual PIN
  //vPinValue = *((float*)(rxBuf+1));

  //if(*rxBuf==VPIN_BLYNK_TERMINAL){
  //  setReceivedVirtualPinValue(*rxBuf, *((float*)(rxBuf+1)));
  //}else{
  setReceivedVirtualPinValue(*rxBuf, *((float*)(rxBuf+1)));
  //}
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat); 
////////////must be implemented!
// for example:
//{
  //	switch(vPinNumber){
  //		case VPIN_STATUS:
  //			STATUS = (int)vPinValueFloat;
  //			break;
		//case VPIN_SetMainCycleInterval:
		//	if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
		//		break;
		//	MainCycleInterval = (int)vPinValueFloat;
		//	timer.deleteTimer(mainTimerId);
		//	mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
		//	break;
		//case VPIN_ManualFloorIn:
		//	tempTargetFloorIn = vPinValueFloat;
		//	break;
  //		default:
  //      #ifdef testmode
  //			Serial.print("! Warning: received unneeded CAN message: VPIN=");
  //			Serial.print(vPinNumber);
  //			Serial.print(" FloatValue=");
  //			Serial.print(vPinValueFloat);
  //			Serial.println();
  //      #endif
  //			return 0;
  //	}
  //	return 1;
//}

float fround(float r, byte dec){
	if(dec>0) for(byte i=0;i<dec;i++) r*=10;
	r=(long)(r+0.5);
	if(dec>0) for(byte i=0;i<dec;i++) r/=10;
	return r;
}

void EEPROM_WriteInt(int p_address, int p_value){
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}
unsigned int EEPROM_ReadInt(int p_address){
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
template <class T> int EEPROM_WriteAnything(int ee, const T& value) //write any type to address ee and return numbytes
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
template <class T> int EEPROM_ReadAnything(int ee, T& value) //read any type from address ee and return numbytes
{
   byte* p = (byte*)(void*)&value;
   unsigned int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return i;
}

void EEPROM_storeValues();
// {  EEPROM.update(VPIN_STATUS,(unsigned char)STATUS);
//   EEPROM.update(VPIN_MainCycleInterval,(unsigned char)(MainCycleInterval/10));
//   EEPROM.update(VPIN_SetPWMch1,(unsigned char)PWMch1);
//   EEPROM.update(VPIN_SetPWMch2,(unsigned char)PWMch2);
//   EEPROM.update(VPIN_SetPWMch3,(unsigned char)PWMch3);
//   EEPROM.update(VPIN_SetPWMch4,(unsigned char)PWMch4);
// }
void EEPROM_restoreValues();
//{  STATUS = EEPROM.read(VPIN_STATUS);
//  
//  int aNewInterval = EEPROM.read(VPIN_MainCycleInterval)*10;
//  if(aNewInterval != 0){
//    MainCycleInterval = aNewInterval;
//  }
//  
//  PWMch1 = EEPROM.read(VPIN_SetPWMch1);
//  PWMch2 = EEPROM.read(VPIN_SetPWMch2);
//  PWMch3 = EEPROM.read(VPIN_SetPWMch3);
//  PWMch4 = EEPROM.read(VPIN_SetPWMch4);
//}

//////////////////////////////////////////////////SETUP///////////////////////////
    //void setup(void) {
    //   EEPROM_restoreValues();
    //   timer.setInterval(1000L*600L, EEPROM_storeValuesOnTimer); //once in 10 min remember critical values
    //#ifdef testmode
    //   Serial.begin(115200);
    //   //#endif
    //
    //   ////////////////Initialize CAN bus MCP2515: mode = the masks and filters disabled.
    //   if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STDEXT - they are the only working modes
    //     ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
    //   else
    //     Serial.println("Error Initializing CAN bus driver MCP2515...");
    //
    //   /////////////////Initialize filters Masks(0-1),Filters(0-5):
    //   unsigned long mask = (0x0100L | CAN_Unit_MASK)<<16;			//0x0F	0x010F0000;
    //   unsigned long filt = (0x0100L | CAN_Unit_FILTER_OUTDT)<<16;	//0x04	0x01040000;
    //   CAN0.init_Mask(0,0,mask);                // Init first mask...
    //   CAN0.init_Filt(0,0,filt);                // Init first filter...
    //   //CAN0.init_Filt(1,0,filt);                // Init second filter...
    //   //CAN0.init_Mask(1,0,mask);                // Init second mask...
    //   //CAN0.init_Filt(2,0,filt);                // Init third filter...
    //   //CAN0.init_Filt(3,0,filt);                // Init fouth filter...
    //   //CAN0.init_Filt(4,0,filt);                // Init fifth filter...
    //   //CAN0.init_Filt(5,0,filt);                // Init sixth filter...
    //
    //   CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
    //   #ifdef testmode
    //   CAN0.setMode(MCP_LOOPBACK);
    //   #endif
    //   pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input
    //
    //   mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
    //}

////////////////////////////////////////////////LOOP////////////////////////////
    //void loop(void) {
    //  timer.run();
    //  checkReadCAN();
    //}