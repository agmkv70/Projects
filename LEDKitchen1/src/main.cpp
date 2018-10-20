#include <SimpleTimer.h>
#include <mcp_can.h>
#include <SPI.h>
#include <QueueList.h>
#include <EEPROM.h>

#define PWMpin1 9  //3 //31k
#define PWMpin2 10 //5 //62k
#define PWMpin3 3  //6 //62k
#define PWMpin4 5  //9 //31k
#define PWMpin1div 8
#define PWMpin2div 8
#define PWMpin3div 8
#define PWMpin4div 8

#define AVreadpin 0 //A0

SimpleTimer timer;
int mainTimerId;

//#define testmode

#define CAN0_INT 8      // INT = pin 9
MCP_CAN CAN0(7);       // CS  = pin 10

unsigned long rxId;
unsigned char dataLen = 0;
unsigned char rxBuf[8];
//filter message types:
#define CAN_MSG_MASK           0xF0
#define CAN_MSG_FILTER_UNITCMD 0x80  //command for special unit
#define CAN_MSG_FILTER_UNITINF 0x40  //some info for special unit
#define CAN_MSG_FILTER_INF     0x20  //some info for everyone who wants it (e.g. outer temperature)
#define CAN_MSG_FILTER_STATIST 0x10  //non-critical statistics
//receive only messages to this unit - include recever id:
#define CAN_Unit_MASK         0x0F
#define CAN_Unit_FILTER_KUHFL 0x01 //Floor temperature tegulation unit
#define CAN_Unit_FILTER_ESPWF 0x02 //ESP8266 WiFi-CAN bridge
#define CAN_Unit_FILTER_ELCT1 0x03 //Electric power control
#define CAN_Unit_FILTER_OUTDT 0x04 //Unit for outdoor temperature

#define Status_Standby	1
#define Status_Auto1	  2 //full auto (tempTargetFloorOut)
#define Status_Auto2	  3 //semi-auto (ManualFloorIn)
#define Status_Manual	  4 //manual valve
#define Status_Warning  5
#define Status_Error	  6
int STATUS = Status_Auto2;//Status_Manual;

#define VPIN_STATUS				0
#define VPIN_ErrCode	  	1
#define VPIN_LEDPower12Voltage  40
#define VPIN_MainCycleInterval  41
#define VPIN_SetMainCycleInterval  42
#define VPIN_SetPWMch1  43
#define VPIN_SetPWMch2  44
#define VPIN_SetPWMch3  45
#define VPIN_SetPWMch4  46
int MainCycleInterval=600, PWMch1=10, PWMch2=10, PWMch3=10, PWMch4=10;

struct CANMessage{ unsigned char vPinNumber; float vPinValueFloat; byte nTries; };
int timerIntervalForNextSendCAN=0;
int CANSendError=0;
QueueList <CANMessage> CANQueue;
void addCANMessage2Queue(unsigned char vPinNumber, float vPinValueFloat);

float fround(float r, byte dec){
	if(dec>0) for(byte i=0;i<dec;i++) r*=10;
	r=(long)(r+0.5);
	if(dec>0) for(byte i=0;i<dec;i++) r/=10;
	return r;
}

char sendVPinCAN(unsigned char vPinNumber, float vPinValueFloat){

	*rxBuf = vPinNumber;
	*(float*)(rxBuf+1) = vPinValueFloat;

	// send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
	byte sndStat = CAN0.sendMsgBuf(0x100 //| CAN_MSG_FILTER_STATIST | CAN_Unit_FILTER_KUHFL
    , 0, 1+sizeof(float), rxBuf);
	if(sndStat == CAN_OK){
		//Serial.println("Message Sent Successfully!");
	} else {
		Serial.print("Error Sending Message by CAN bus!.. pin=");
    Serial.println(vPinNumber);
    return 0;
	}
  return 1;
}

//SENDNEXT CAN ////////////////////////////////////////////////////////////////////////////
void sendNextCANMessage(){ 

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
    return; //nothing to send is also good
  }

  //queue is not empty:
  CANMessage mes = CANQueue.pop();
  
  char res = sendVPinCAN( mes.vPinNumber, mes.vPinValueFloat );

  if(res == CAN_OK){
    ;//no pushing back = drop this message
  }else{
    mes.nTries++;
		if( mes.nTries > 10 ){
      //no pushing back = drop this message
      CANSendError = res;
		}else{
			CANQueue.push(mes);
		}
	}

  if( CANQueue.isEmpty() ){
    timerIntervalForNextSendCAN=0;
  }else{ //not empty - try again soon:
    timerIntervalForNextSendCAN = timer.setTimeout( 5, sendNextCANMessage ); //5 millis try interval
  }
}

//ADD CAN /////////////////////////////////////////////////////////////////////////////////
void addCANMessage2Queue(unsigned char vPinNumber, float vPinValueFloat){ 
  CANQueue.push( CANMessage{ vPinNumber, vPinValueFloat, 0} );
  if(timerIntervalForNextSendCAN==0){
    timerIntervalForNextSendCAN = timer.setTimeout(5,sendNextCANMessage); //5 millis try interval
  }
  
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat); //decl

//checkRead CAN ///////////////////////////////////////////////////////////////////////////
void checkReadCAN() {
  if(digitalRead(CAN0_INT)){ //HIGH = no CAN received messages in buffer
	  return;
  }
	#ifdef testmode
  Serial.println("Found CAN message: ");
	#endif
  //If CAN0_INT pin is LOW, read receive buffer:
  int canRes=CAN0.readMsgBuf(&rxId, &dataLen, rxBuf);
  if( canRes != CAN_OK) // Read data: len = data length, buf = data byte(s)
	{	
    #ifdef testmode
    Serial.print("canreadErr=");
    Serial.println(canRes);
	  #endif
    return;
  }

  #ifdef testmode
  Serial.print(dataLen);
  Serial.print(": pin=");
  Serial.print(*rxBuf);
  Serial.print(" val=");
  Serial.print(*((float*)(rxBuf+1)));
  // for(byte i = 0; i<dataLen; i++){
  //   Serial.print(rxBuf[i]);
  //   Serial.print(" ");
  // }
  Serial.println();
	#endif

  if(dataLen<5)
	  return; //wee need at least 5 bytes (1 = number of VPIN, 2-5 = float value (4 bytes))

  //vPinNumber = *rxBuf; //first byte is Number of Virtual PIN
  //vPinValue = *((float*)(rxBuf+1));

  setReceivedVirtualPinValue(*rxBuf, *((float*)(rxBuf+1)));
}

void MainCycle_StartEvent(){
  //digitalWrite(13,HIGH);

  float avolt = ((float)analogRead(AVreadpin)/1000*15.492773); //reading of 12V
    
  #ifdef testmode
		Serial.print("a0volt = ");
		Serial.println(avolt);
  #endif

  addCANMessage2Queue(VPIN_LEDPower12Voltage,fround(avolt,2)); //rounded 0.0 value
	//digitalWrite(13,LOW);
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
	switch(vPinNumber){
		case VPIN_STATUS:
			STATUS = (int)vPinValueFloat;
			break;
		case VPIN_SetMainCycleInterval:
			if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
				break;
			MainCycleInterval = (int)vPinValueFloat;
			timer.deleteTimer(mainTimerId);
			mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
			break;
		case VPIN_SetPWMch1:  PWMch1  = vPinValueFloat; analogWrite(PWMpin1,PWMch1); break;
		case VPIN_SetPWMch2:  PWMch2  = vPinValueFloat; analogWrite(PWMpin2,PWMch2); break;
		case VPIN_SetPWMch3:  PWMch3  = vPinValueFloat; analogWrite(PWMpin3,PWMch3); break;
		case VPIN_SetPWMch4:  PWMch4  = vPinValueFloat; analogWrite(PWMpin4,PWMch4); break;
		default:
			return 0;
	}
	return 1;
}

void EEPROM_storeValuesOnTimer(){
  EEPROM.update(VPIN_STATUS,(unsigned char)STATUS);
  EEPROM.update(VPIN_MainCycleInterval,(unsigned char)(MainCycleInterval/10));

  EEPROM.update(VPIN_SetPWMch1,(unsigned char)PWMch1);
  EEPROM.update(VPIN_SetPWMch2,(unsigned char)PWMch2);
  EEPROM.update(VPIN_SetPWMch3,(unsigned char)PWMch3);
  EEPROM.update(VPIN_SetPWMch4,(unsigned char)PWMch4);

}
void EEPROM_restoreValues(){
  STATUS = EEPROM.read(VPIN_STATUS);
  
  int aNewInterval = EEPROM.read(VPIN_MainCycleInterval)*10;
  if(aNewInterval != 0){
    MainCycleInterval = aNewInterval;
  }
  
  PWMch1 = EEPROM.read(VPIN_SetPWMch1);
  PWMch2 = EEPROM.read(VPIN_SetPWMch2);
  PWMch3 = EEPROM.read(VPIN_SetPWMch3);
  PWMch4 = EEPROM.read(VPIN_SetPWMch4);
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

/** Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0 (delay,millis)
 *   - Pins 9 and 10 are paired on timer1 (servo)
 *   - Pins 3 and 11 are paired on timer2 (tone)
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}

void setup() {

  EEPROM_restoreValues();
  timer.setInterval(1000L*600L, EEPROM_storeValuesOnTimer); //once in 10 min remember critical values

  #ifdef testmode
	Serial.begin(115200);
  Serial.println("Start");
	#endif

  pinMode(PWMpin1,OUTPUT);
  pinMode(PWMpin2,OUTPUT);
  pinMode(PWMpin3,OUTPUT);
  pinMode(PWMpin4,OUTPUT);
  //digitalWrite(PWMpin1,0);

  setPwmFrequency(PWMpin1,PWMpin1div);
  setPwmFrequency(PWMpin2,PWMpin2div);
  setPwmFrequency(PWMpin3,PWMpin3div);
  //setPwmFrequency(PWMpin4,PWMpin4div);

  analogWrite(PWMpin1,PWMch1);
  analogWrite(PWMpin2,PWMch2);
  analogWrite(PWMpin3,PWMch3);
  analogWrite(PWMpin4,PWMch4);

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
  //if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
		#ifdef testmode
		Serial.println("Error Initializing CAN bus driver MCP2515...");
		#endif
	}

  #ifdef testmode
	CAN0.setMode(MCP_LOOPBACK);
	#endif
	#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
	#endif
  pinMode(CAN0_INT, INPUT);  // Configuring CAN0_INT pin for input

  //pinMode(13,OUTPUT);//led

  mainTimerId = timer.setInterval(1000L*MainCycleInterval, MainCycle_StartEvent); //start regularly
}

int i=0;
int way=1;
void loop() {
    //system events:
    timer.run();
    checkReadCAN();

    ////////other code:
    //analogWrite(PWMpin1,255);
    //delay(20);

    //analogWrite(PWMpin1,i);
    //analogWrite(PWMpin2,i);
    //analogWrite(PWMpin3,i);
    //analogWrite(PWMpin4,i);
    //delay(20);
    
    //i=i+way;
    //if(i>55)
    //{ way=-1; i=i+way; }
    //if(i<0)
    //{ way=1; i=i+way; }
    
}

