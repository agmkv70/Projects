//NIK_can.h
//all for can

#include <SimpleTimer.h>
#include <mcp_can.h>
#include <SPI.h>
#include <QueueList.h>

//#define testmode

//#define CAN_PIN_INT  9        // INT = pin 9
MCP_CAN CAN0(CAN_PIN_CS);       // CS  = pin 10

unsigned long rxId;
unsigned char dataLen = 0;
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

SimpleTimer timer;

struct CANMessage{ long mesID; unsigned char vPinNumber; float vPinValueFloat; byte nTries; };
int timerIntervalForNextSendCAN=0;
int CANQueueError=0; 
#define CANQueueErrorOverflow 7777
static int CANQueueMaxLength=30;
QueueList <CANMessage> CANQueue;

///////////////////////////////////////CAN//////////////////////////////////////////
char sendVPinCAN(long mesID, unsigned char vPinNumber, float vPinValueFloat){ //transfer by CAN

	*rxBuf = vPinNumber;
	*(float*)(rxBuf+1) = vPinValueFloat;

	// send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
	byte sndStat = CAN0.sendMsgBuf((0x100L | mesID)<<16, 0, 1+sizeof(float), rxBuf);
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

  char res = sendVPinCAN( mes.mesID, mes.vPinNumber, mes.vPinValueFloat );

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
//addCANMessage2Queue(CAN_MSG_FILTER_STATIST | CAN_Unit_FILTER_ESPWF, VPIN_OutdoorTemp, tempOutdoor);
void addCANMessage2Queue(long mesID, unsigned char vPinNumber, float vPinValueFloat){
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

  #ifdef testmode
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

  if(dataLen<=5)
    return; //we need at least 5 bytes (1 = number of VPIN, 2-5 = float value (4 bytes))
  //vPinNumber = *rxBuf; //first byte is Number of Virtual PIN
  //vPinValue = *((float*)(rxBuf+1));

  setReceivedVirtualPinValue(*rxBuf, *((float*)(rxBuf+1)));
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

//////////////////////////////////////////////////SETUP///////////////////////////
    //void setup(void) {
    //   //#ifdef testmode
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
    //   pinMode(CAN0_INT, INPUT);  // Configuring CAN0_INT pin for input
    //
    //   mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
    //}

////////////////////////////////////////////////LOOP////////////////////////////
    //void loop(void) {
    //  timer.run();
    //  checkReadCAN();
    //}