///FloorTemp///
//For DS18B20 connected to separate pins (no need to know adresses, means interchangebility):

#include <OneWire.h>

//#define testmode

#define CAN_PIN_INT 9    
#define CAN_PIN_CS 10 

#include <NIK_defs.h>
#include <NIK_can.h>

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
int ErrorMeasTemp=0; //flag of temp sensor lag - don't change state if can't read inputs properly

int mainTimerId, boilerPWMTimerId;

float fround(float r, byte dec){
	if(dec>0) for(byte i=0;i<dec;i++) r*=10;
	r=(long)(r+0.5);
	if(dec>0) for(byte i=0;i<dec;i++) r/=10;
	return r;
}

void BoilerPIDEvaluation(){
	float delta = BoilerTargetTemp - tempBoiler;
	float tmpBoilerPID_Isum = BoilerPID_Isum + delta;

	float P = BoilerPID_Kp * delta;
	float I = BoilerPID_Ki * tmpBoilerPID_Isum;
	float D = BoilerPID_Kd * (delta - BoilerPID_prevDelta);
	BoilerPID_prevDelta = delta;

  BoilerPower = BoilerPower + P + I + D;
	
	if(BoilerPower<=0) BoilerPower=0;
	else if(BoilerPower>=10) BoilerPower=10;
	else BoilerPID_Isum = tmpBoilerPID_Isum; //sum Integral part of PID only if our result is not saturated

	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_P, fround(P,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_I, fround(I,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_D, fround(D,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPower, fround(BoilerPower,1));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerTargetTempGraph, fround(BoilerTargetTemp,1));
}

void BoilerPWMTimerEvent(){ //PWM = ON at the beginning, OFF at the end of cycle
	long millisFromStart = millis()-BoilerPWMCycleStart;
		
	if( millisFromStart >= BoilerPeriodMillis ){ //time to START:
		if( !ErrorMeasTemp )
			BoilerPIDEvaluation(); //once: on boiler pwm cycle start and only if temperature is really read
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
			addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPowerOnOff, BoilerPowerCurrentStateOnOff);
		}
	}else{ //BoilerPowerCurrentStateOnOff==1 //we are ON - we can only turn off:
			if( millisFromStart >= millisOn ){ //time to TURN OFF:
				if(millisOn < BoilerPeriodMillis)	BoilerPowerCurrentStateOnOff=0;
				addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPowerOnOff, BoilerPowerCurrentStateOnOff);
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
		#ifdef testmode
		Serial.println();
		Serial.print("!ERROR: temp sensor CRC failure - ");
		Serial.println(dname);
		#endif
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

void MainCycle_StartEvent() {

	switch(boardSTATUS){
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

	ErrorMeasTemp=0;
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

	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_Boiler,fround(tempBoiler,1)); //rounded 0.0 value
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_FloorIn,fround(tempFloorIn,1)); //rounded 0.0 value
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_FloorOut,fround(tempFloorOut,1)); //rounded 0.0 value

  if(!ErrorMeasTemp){
	    float needChangeInTemp=0, needChangeSeconds=0;

		if(boardSTATUS==Status_Auto1){	//change FloorIn, depending on FloorOut:
			needChangeInTemp = (tempTargetFloorOut - tempFloorOut)*0.1; //slowly approaching
		}

		if(boardSTATUS==Status_Auto1 || boardSTATUS==Status_Auto2){	//try to achieve needed FloorIn fast:
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

		  if( boardSTATUS==Status_Auto1 || boardSTATUS==Status_Auto2 ) {
			  ValveStartDecrease();
			  timer.setTimeout(needChangeMillis, ValveStop);
		  }
		}else if(needChangeSeconds > 0){

		  needChangeMillis = needChangeSeconds * (float)Valve_OneMoveStepMillis;

		  //addCANMessage2Queue(VPIN_MotorValvePlus,needChangeMillis);

		  if( boardSTATUS==Status_Auto1 || boardSTATUS==Status_Auto2 ) {
			ValveStartIncrease();
			timer.setTimeout(needChangeMillis, ValveStop);
		  }
		}
	}
	//addCANMessage2Queue(VPIN_BoilerPower,fround(BoilerPower,2); //rounded 0.00 value
}

void checkReadCAN() {
  if(digitalRead(CAN_PIN_INT)){ //HIGH = no CAN received messages in buffer
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
			boardSTATUS = (int)vPinValueFloat;
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
			#ifdef testmode
			Serial.print("! Warning: received unneeded CAN message: VPIN=");
			Serial.print(vPinNumber);
			Serial.print(" FloatValue=");
			Serial.print(vPinValueFloat);
			Serial.println();
      #endif
			return 0;
	}
	return 1;
}

////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
	boardSTATUS = Status_Manual; //init

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
  unsigned long mask  = (0x0100L | CAN_Unit_MASK | CAN_MSG_MASK)<<16;			//0x0F	0x010F0000;
  unsigned long filt0 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_UNITCMD)<<16;	//0x04	0x01040000;
  unsigned long filt1 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_INF)<<16;	//0x04	0x01040000;
  CAN0.init_Mask(0,0,mask);                // Init first mask...
  CAN0.init_Filt(0,0,filt0);                // Init first filter...
  #ifdef testmode
  CAN0.init_Filt(1,0,filt1);                // Init second filter...
  #endif
  CAN0.init_Mask(1,0,0x01FFFFFF);                // Init second mask...
  CAN0.init_Filt(2,0,0x01FFFFFF);                // Init third filter...
  //CAN0.init_Filt(3,0,filt);                // Init fouth filter...
  //CAN0.init_Filt(4,0,filt);                // Init fifth filter...
  //CAN0.init_Filt(5,0,filt);                // Init sixth filter...
  
	#ifdef testmode
	CAN0.setMode(MCP_LOOPBACK);
	#endif
	#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data
	#endif
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input

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
