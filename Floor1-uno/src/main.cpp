///FloorTemp///
//For DS18B20 connected to separate pins (no need to know adresses, means interchangebility):

#include <OneWire.h>
#include <SimpleTimer.h>
#include <mcp_can.h>
#include <SPI.h>
#include <QueueList.h>

//#define testmode

#define CAN0_INT 9      // INT = pin 2
MCP_CAN CAN0(10);       // CS  = pin 10

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
#define VPIN_Boiler				2
#define VPIN_FloorIn			3
#define VPIN_ManualFloorIn		4 	//semi-auto
#define VPIN_FloorOut			5
#define VPIN_tempTargetFloorOut	6	//full auto
#define VPIN_ManualMotorValveMinus	7 	//manual -
#define VPIN_ManualMotorValvePlus	8 	//manual +
#define VPIN_MotorValveMinus	9 		//unit's decision to move - (in automatic mode it's made automatically)
#define VPIN_MotorValvePlus		10 		//unit's decision to move + (in automatic mode it's made automatically)
#define VPIN_MainCycleInterval  11
#define VPIN_SetMainCycleInterval  12

#define VPIN_OutdoorTemp  13 //other Uno module on CAN

#define VPIN_SetBoilerPowerPeriodMinutes  14
#define VPIN_BoilerPowerOnOff 15
#define VPIN_BoilerPower 16
#define VPIN_BoilerPID_Kp 17 
#define VPIN_BoilerPID_Ki 18 
#define VPIN_BoilerPID_Kd 19 
#define VPIN_BoilerPID_P 20 //proportional
#define VPIN_BoilerPID_I 21 //integral
#define VPIN_BoilerPID_D 22 //differential
#define VPIN_BoilerTargetTemp 23
#define VPIN_SetBoilerPID_Isum_Zero 24
#define VPIN_BoilerTargetTempGraph 25

#define ValveMinusPin 6
#define ValvePlusPin  7
#define LED_PIN 13

#define BOILER_ON_PIN 14 //(analog 0 same as digital 14)
float BoilerPower=0; //Boiler power on time factor 0.0 .. 10.0 float
int BoilerPowerCurrentStateOnOff=0; //0=on or 1=off
int BoilerPowerPeriodMinutes=10; //period of PWM
long BoilerPeriodMillis=60000L; //period of PWM in millis
long BoilerPWMCycleStart=0; //last cycle start
float BoilerTargetTemp=23;
float BoilerPID_Kp=0.6, BoilerPID_Ki=0.02, BoilerPID_Kd=0.9;
float BoilerPID_Isum=0, BoilerPID_prevDelta=0;

//initdiff: 22.06 21.44 22.12
OneWire  TempDS_Boiler(2);  // on pin 2 (a 4.7K resistor is necessary)
OneWire  TempDS_FloorIn(3);  // on pin 3 (a 4.7K resistor is necessary)
OneWire  TempDS_FloorOut(4);  // on pin 4 (a 4.7K resistor is necessary)
//OneWire  TempDS_Ambient(5);  // on pin 5 (a 4.7K resistor is necessary)

int MainCycleInterval=30; //изредка 60
float tempBoiler=20, offsetBoiler=0;
float tempFloorIn=20, offsetFloorIn=0.562;
float tempFloorOut=20, offsetFloorOut=-0.062;
//float tempAmbient=20;

float tempTargetFloorIn=23;
float tempTargetFloorOut = 20;
float tempMaxFloorIn     = 29;
float tempMaxFloorInOutDiff  = 8;
unsigned long Valve_OneMoveStepMillis = 1000L;
float maxMoveSec = 10;
float minValveStep = 1;
float cumulatedUndersteps = 0;

SimpleTimer timer;
int mainTimerId, boilerPWMTimerId;

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

void BoilerPIDEvaluation(){
	float delta = BoilerTargetTemp - tempBoiler;
	BoilerPID_Isum = BoilerPID_Isum + delta;

	float P = BoilerPID_Kp * delta;
	float I = BoilerPID_Ki * BoilerPID_Isum;
	float D = BoilerPID_Kd * (delta - BoilerPID_prevDelta);
	BoilerPID_prevDelta = delta;

  BoilerPower = BoilerPower + P + I + D;
	
	if(BoilerPower<0) BoilerPower=0;
	else if(BoilerPower>10) BoilerPower=10;

	addCANMessage2Queue(VPIN_BoilerPID_P, fround(P,2));
	addCANMessage2Queue(VPIN_BoilerPID_I, fround(I,2));
	addCANMessage2Queue(VPIN_BoilerPID_D, fround(D,2));
	addCANMessage2Queue(VPIN_BoilerPower, fround(BoilerPower,1));
	addCANMessage2Queue(VPIN_BoilerTargetTempGraph, fround(BoilerTargetTemp,1));
}

void BoilerPWMTimerEvent(){ //PWM = ON at the beginning, OFF at the end of cycle
	long millisFromStart = millis()-BoilerPWMCycleStart;
		
	if( millisFromStart >= BoilerPeriodMillis ){ //time to START:
		BoilerPIDEvaluation(); //once: on boiler pwm cycle start
	  BoilerPWMCycleStart = millis();
		millisFromStart=0;
	}

	//evaluate millisOn (so to say, transformation of BoilerPower):
	long millisOn = (BoilerPower/10*(float)BoilerPeriodMillis); //sould be on in cycle
	if(BoilerPower<0.1f) millisOn = 0;
	else if(BoilerPower>9.9f) millisOn = BoilerPeriodMillis;

	if(BoilerPowerCurrentStateOnOff==0){ //we are off - we can only turn on:
		if( millisFromStart < millisOn ){ //time to START:
			if(millisOn > 0) BoilerPowerCurrentStateOnOff=1;
			addCANMessage2Queue(VPIN_BoilerPowerOnOff, BoilerPowerCurrentStateOnOff);
		}
	}else{ //BoilerPowerCurrentStateOnOff==1 //we are ON - we can only turn off:
			if( millisFromStart >= millisOn ){ //time to TURN OFF:
				if(millisOn < BoilerPeriodMillis)	BoilerPowerCurrentStateOnOff=0;
				addCANMessage2Queue(VPIN_BoilerPowerOnOff, BoilerPowerCurrentStateOnOff);
			}
	}

  if(BoilerPowerCurrentStateOnOff==1)
		digitalWrite(BOILER_ON_PIN,HIGH);
	else
		digitalWrite(BOILER_ON_PIN,LOW);
}

void ValveStop() {
  digitalWrite(ValveMinusPin,HIGH);  //off
  digitalWrite(ValvePlusPin, HIGH);  //off
}
void ValveStartDecrease() {
  digitalWrite(ValvePlusPin, HIGH);  //off
  digitalWrite(ValveMinusPin,LOW);   //on
}
void ValveStartIncrease() {
  digitalWrite(ValveMinusPin,HIGH);  //off
  digitalWrite(ValvePlusPin, LOW);   //on
}

void TempDS_AllStartConvertion() {
  TempDS_Boiler.reset();
  TempDS_Boiler.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_Boiler.write(0x44); // start conversion
  TempDS_FloorIn.reset();
  TempDS_FloorIn.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_FloorIn.write(0x44); // start conversion
  TempDS_FloorOut.reset();
  TempDS_FloorOut.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_FloorOut.write(0x44); // start conversion
  //TempDS_Ambient.reset();
  //TempDS_Ambient.write(0xCC); //skip rom - next command to all //ds.select(addr);
  //TempDS_Ambient.write(0x44); // start conversion
}

byte TempDS_GetTemp(OneWire *ds, String dname, float *temp) { //interface object and sensor name, returns 1 if OK
  byte data[12];

  ds->reset(); //перезагрузка с отключением питания (делаем перед запросом на замер, тут не надо)
  ds->write(0xCC); //Skip ROM - next command to all //ds.select(addr);
  ds->write(0xBE); //Read Scratchpad

  //Serial.print(dname);
  //Serial.print(" :");
  for (byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds->read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  if(data[8]!=OneWire::crc8(data, 8)) {
	Serial.println("");
	Serial.print("!ERROR: temp sensor CRC failure - ");
	Serial.println(dname);
	return 0; //crc failure
  }

  // Calculate temperature value
  *temp = (float)( (data[1] << 8) + data[0] )*0.0625F;

	#ifdef testmode
  Serial.print(" ");
  Serial.print(*temp);
	#endif

  return 1; //OK
}
void MainCycle_ReadTempEvent(); //declaration

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

//SENDNEXT ////////////////////////////////////////////////////////////////////////////
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

//ADD /////////////////////////////////////////////////////////////////////////////////
void addCANMessage2Queue(unsigned char vPinNumber, float vPinValueFloat){ 
  CANQueue.push( CANMessage{ vPinNumber, vPinValueFloat, 0} );
  if(timerIntervalForNextSendCAN==0){
    timerIntervalForNextSendCAN = timer.setTimeout(5,sendNextCANMessage); //5 millis try interval
  }
  
}

void MainCycle_StartEvent() {

	switch(STATUS){
		case Status_Standby:
			return;	//skip standby state
	}

	TempDS_AllStartConvertion();
	#ifdef testmode
	Serial.println("Start coversion... ");
	#endif

	timer.setTimeout(1000L, MainCycle_ReadTempEvent); //start once after timeout
}

void MainCycle_ReadTempEvent() {

	int ErrorMeasTemp=0;
	if( !TempDS_GetTemp(&TempDS_Boiler,"BOILER",&tempBoiler) ){
	 ErrorMeasTemp++;
	}
	if( !TempDS_GetTemp(&TempDS_FloorIn,"FLOORIN",&tempFloorIn) ){
	 ErrorMeasTemp++;
	}
	if( !TempDS_GetTemp(&TempDS_FloorOut,"FLOOROUT",&tempFloorOut) ){
	 ErrorMeasTemp++;
	}
	//if( !TempDS_GetTemp(&TempDS_Ambient,"AMBIENT",&tempAmbient) ){
	// ErrorMeasTemp++;
	//
	#ifdef testmode
    Serial.println();
	#endif
  tempBoiler += offsetBoiler;
  tempFloorIn += offsetFloorIn;
  tempFloorOut += offsetFloorOut;


	addCANMessage2Queue(VPIN_Boiler,fround(tempBoiler,1)); //rounded 0.0 value
	addCANMessage2Queue(VPIN_FloorIn,fround(tempFloorIn,1)); //rounded 0.0 value
	addCANMessage2Queue(VPIN_FloorOut,fround(tempFloorOut,1)); //rounded 0.0 value

  if(!ErrorMeasTemp){
	    float needChangeInTemp=0, needChangeSeconds=0;

		if(STATUS==Status_Auto1){	//change FloorIn, depending on FloorOut:
			needChangeInTemp = (tempTargetFloorOut - tempFloorOut)*0.1; //slowly approaching
		}

		if(STATUS==Status_Auto1 || STATUS==Status_Auto2){	//try to achieve needed FloorIn fast:
			needChangeInTemp = tempTargetFloorIn - tempFloorIn;
			needChangeSeconds = needChangeInTemp*3;
		}

		needChangeSeconds += cumulatedUndersteps;
		cumulatedUndersteps=0;

		//check min, max restrictions:
		if(tempFloorIn >= tempMaxFloorIn && needChangeSeconds>0){
		  needChangeSeconds = 0;
		}
		if(tempFloorIn-tempFloorOut >= tempMaxFloorInOutDiff && needChangeSeconds>0){
		  needChangeSeconds = 0;
		}
		if(tempFloorIn-tempFloorOut > tempMaxFloorInOutDiff){
		  needChangeSeconds = -(tempMaxFloorInOutDiff - (tempFloorIn-tempFloorOut))*3;
		}
    
		#ifdef testmode
		Serial.print("motor calc sec = ");
		Serial.print(needChangeSeconds);
		#endif

		if( needChangeSeconds < -maxMoveSec) //too left
			needChangeSeconds = -maxMoveSec;
		if(needChangeSeconds > maxMoveSec)   //too right
			needChangeSeconds = maxMoveSec;

		if( 0 < -needChangeSeconds && -needChangeSeconds < minValveStep){ //too small step left
			cumulatedUndersteps -= needChangeSeconds;
			needChangeSeconds = 0;
		}
		if( 0 < needChangeSeconds && needChangeSeconds < minValveStep){ //too small step right
			cumulatedUndersteps += needChangeSeconds;
			needChangeSeconds = 0;
		}

		#ifdef testmode
		Serial.print("  motor real millis = ");
		Serial.println((long)(needChangeSeconds*1000));
		#endif
		long needChangeMillis;

		//start moving valve:
		if(needChangeSeconds < 0){

		  needChangeMillis = (-needChangeSeconds) * (float)Valve_OneMoveStepMillis;

		  //addCANMessage2Queue(VPIN_MotorValveMinus,needChangeMillis);

		  if( STATUS==Status_Auto1 || STATUS==Status_Auto2 ) {
			  ValveStartDecrease();
			  timer.setTimeout(needChangeMillis, ValveStop);
		  }
		}else if(needChangeSeconds > 0){

		  needChangeMillis = needChangeSeconds * (float)Valve_OneMoveStepMillis;

		  //addCANMessage2Queue(VPIN_MotorValvePlus,needChangeMillis);

		  if( STATUS==Status_Auto1 || STATUS==Status_Auto2 ) {
			ValveStartIncrease();
			timer.setTimeout(needChangeMillis, ValveStop);
		  }
		}
	}
	
	//addCANMessage2Queue(VPIN_BoilerPower,fround(BoilerPower,2); //rounded 0.00 value
}
char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat); //decl

void checkReadCAN() {
  if(digitalRead(CAN0_INT)){ //HIGH = no CAN received messages in buffer
	  return;
  }
	#ifdef testmode
  Serial.println("Found CAN message: ");
	#endif
  //If CAN0_INT pin is LOW, read receive buffer:
  if(CAN0.readMsgBuf(&rxId, &dataLen, rxBuf) != CAN_OK) // Read data: len = data length, buf = data byte(s)
		return;

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
		case VPIN_ManualFloorIn:
			tempTargetFloorIn = vPinValueFloat;
			break;
		case VPIN_tempTargetFloorOut:
			tempTargetFloorOut = vPinValueFloat;
			break;
		case VPIN_ManualMotorValveMinus:
			if(vPinValueFloat==0) ValveStop(); else ValveStartDecrease();
			break;
		case VPIN_ManualMotorValvePlus:
			if(vPinValueFloat==0) ValveStop(); else ValveStartIncrease();
			break;
		case VPIN_SetBoilerPowerPeriodMinutes:
			BoilerPowerPeriodMinutes = vPinValueFloat;
			BoilerPeriodMillis = (long)BoilerPowerPeriodMinutes*60000L;
			break;
		case VPIN_BoilerPID_Kp: BoilerPID_Kp = vPinValueFloat; break;
		case VPIN_BoilerPID_Ki: BoilerPID_Ki = vPinValueFloat; break;
		case VPIN_BoilerPID_Kd: BoilerPID_Kd = vPinValueFloat; break;
		case VPIN_BoilerPower:  BoilerPower  = vPinValueFloat; break;
		case VPIN_BoilerTargetTemp: BoilerTargetTemp = vPinValueFloat; break;
		case VPIN_SetBoilerPID_Isum_Zero: BoilerPID_Isum=0; break;
		default:
			return 0;
	}
	return 1;
}

////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
  #ifdef testmode
	Serial.begin(115200);
	#endif

  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

	pinMode(BOILER_ON_PIN,OUTPUT);
  digitalWrite(BOILER_ON_PIN,LOW); //turn off BOILER
	
  //turn off relays:
  pinMode(ValveMinusPin,OUTPUT);
  pinMode(ValvePlusPin,OUTPUT);
  ValveStop(); //initial

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
		#ifdef testmode
		Serial.println("Error Initializing CAN bus driver MCP2515...");
		#endif
	}

  //initialize filters Masks(0-1),Filters(0-5):
  //CAN0.init_Mask(0,0,0x010F0000);                // Init first mask...
  //CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
  //CAN0.init_Filt(1,0,0x01010000);                // Init second filter...
  //CAN0.init_Mask(1,0,0x010F0000);                // Init second mask...
  //CAN0.init_Filt(2,0,0x01030000);                // Init third filter...
  //CAN0.init_Filt(3,0,0x01040000);                // Init fouth filter...
  //CAN0.init_Filt(4,0,0x01060000);                // Init fifth filter...
  //CAN0.init_Filt(5,0,0x01070000);                // Init sixth filter...
  
	#ifdef testmode
	CAN0.setMode(MCP_LOOPBACK);
	#endif
	#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
	#endif
  pinMode(CAN0_INT, INPUT);  // Configuring CAN0_INT pin for input


  mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
	delay(100); //for two timers not at once
	boilerPWMTimerId = timer.setInterval(1000L, BoilerPWMTimerEvent); //1 sec pwm discretion
	BoilerPWMCycleStart = millis();
}

////////////////////////////////////////////////LOOP////////////////////////////
void loop(void) {

  timer.run();
  checkReadCAN();

}
