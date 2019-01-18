///VentHeater///

#include <OneWire.h>

#define testmode

//K-thermocouple pins:
#define MAX6675_CS   10
#define MAX6675_SO   12
#define MAX6675_SCK  13

#define CAN_PIN_INT 9    
#define CAN_PIN_CS 10 

#include <NIK_defs.h>
#include <NIK_can.h>

#define ValveMinusPin 7
#define ValvePlusPin  8
#define LED_PIN 13

#define BOILER_ON_PIN 6 //D6 on nano //uno was 14 //(analog 0 same as digital 14)

int VALVESTATUS=0, //vpin37 //0 1    manual / auto floorIn
		PIDSTATUS=0;	 //vpin36 //0 1 2  manual BoilerPower / auto BoilerTemp / auto HomeTemp

float BoilerPower=0; //Boiler power on time factor 0.0 .. 10.0 float
int BoilerPowerCurrentStateOnOff=0; //0=on or 1=off
int BoilerPowerPeriodMinutes=8; //period of PWM
long BoilerPeriodMillis=60000L; //period of PWM in millis
long BoilerPWMCycleStart=0; //last cycle start
float BoilerTargetTemp=23;
float BoilerPID_KCoef=0.1, //for convenience multiply all BoilerPID_K
	BoilerPID_Kp=6, BoilerPID_Ki=0.2, BoilerPID_Kd=9;
float BoilerPID_Isum=0, BoilerPID_prevDelta=0;

float HomeTargetTemp=21;
float BoilerTargetTemp_MIN=21;
float BoilerTargetTemp_MAX=29;
float HomePID_KCoef=0.01, //for convenience multiply all HomePID_K
	HomePID_Kp=6, HomePID_Ki=1, HomePID_Kd=9;
float HomePID_Isum=0, HomePID_prevDelta=0;

//initdiff: 22.06 21.44 22.12
OneWire  TempDS_Boiler(2);  // on pin 2 (a 4.7K resistor to 5V is necessary)
OneWire  TempDS_FloorIn(3);  // on pin 3 (a 4.7K resistor to 5V is necessary)
OneWire  TempDS_FloorOut(4);  // on pin 4 (a 4.7K resistor to 5V is necessary)
OneWire  TempDS_Home(5);  // on pin 5 (a 4.7K resistor to 5V is necessary)

#ifdef testmode
int MainCycleInterval=10; //изредка 60
#endif
#ifndef testmode
int MainCycleInterval=60; //изредка 60
#endif
int eepromVIAddr=1000,eepromValueIs=7750+2; //if this is in eeprom, then we got valid values, not junk
// 21.12 20.94 21.19
// 21.06 20.87 21.12
float tempBoiler=20, offsetBoiler=0;
float tempFloorIn=20, offsetFloorIn=0;//0.562;
float tempFloorOut=20, offsetFloorOut=0;//-0.062;
float tempHome=0, offsetHome=0;

float tempTargetFloorIn  = 24;
float tempTargetFloorOut = 22.5;
float tempMaxFloorIn     = 29;
float tempMaxFloorInOutDiff  = 8;
unsigned long Valve_OneMoveStepMillis = 1000L;
float maxMoveSec = 10;
float minValveStep = 1;
float cumulatedUndersteps = 0;
int ErrorMeasTemp=0; //flag of temp sensor lag - don't change state if can't read inputs properly
int ErrorTempBoiler=0,ErrorTempFloorIn=0,ErrorTempFloorOut=0,ErrorTempHome=0;

int mainTimerId, boilerPWMTimerId, KTCtimerID;

float fround(float r, byte dec){
	if(dec>0) for(byte i=0;i<dec;i++) r*=10;
	r=(long)(r+0.5);
	if(dec>0) for(byte i=0;i<dec;i++) r/=10;
	return r;
}

float readThermocoupleMAX6675() {
  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  { // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }
	
  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;
  // The remaining bits are the number of 0.25 degree (C) counts
  return (float)v*0.25; //returning float
}

void KTCtimer_StartEvent(){
	Serial.print("KTC_MAX6675 = ");
	Serial.println(readThermocoupleMAX6675());
}

void HomePIDEvaluation(){
	float delta = HomeTargetTemp - tempHome;
	
	//sum Integral part of PID only if our result is not saturated:
	if(BoilerTargetTemp<=BoilerTargetTemp_MIN && delta<0)
		; //don't go too far down
	else if(BoilerTargetTemp>=BoilerTargetTemp_MAX && delta>0)
		; //don't go too far up
	else
		HomePID_Isum = HomePID_Isum + delta;

	float P = (HomePID_Kp * HomePID_KCoef) * delta;
	float I = (HomePID_Ki * HomePID_KCoef) * HomePID_Isum;
	float D = (HomePID_Kd * HomePID_KCoef) * (delta - HomePID_prevDelta);
	HomePID_prevDelta = delta;

  BoilerTargetTemp = BoilerTargetTemp + P + I + D;
	
	if(BoilerTargetTemp < BoilerTargetTemp_MIN){
		BoilerTargetTemp = BoilerTargetTemp_MIN;
	}else if(BoilerTargetTemp > BoilerTargetTemp_MAX){
		BoilerTargetTemp = BoilerTargetTemp_MAX;
	}

	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HomePID_P, fround(P,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HomePID_I, fround(I,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HomePID_D, fround(D,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HomeTargetTempGraph, fround(HomeTargetTemp,1));
}

void BoilerPIDEvaluation(){
	float delta = BoilerTargetTemp - tempBoiler;
	
	//sum Integral part of PID only if our result is not saturated:
	if(BoilerPower<=0 && delta<0)
		; //don't go too far down
	else if(BoilerPower>=10 && delta>0)
		; //don't go too far up
	else
		BoilerPID_Isum = BoilerPID_Isum + delta;

	float P = (BoilerPID_Kp * BoilerPID_KCoef) * delta;
	float I = (BoilerPID_Ki * BoilerPID_KCoef) * BoilerPID_Isum;
	float D = (BoilerPID_Kd * BoilerPID_KCoef) * (delta - BoilerPID_prevDelta);
	BoilerPID_prevDelta = delta;

  BoilerPower = BoilerPower + P + I + D;
	
	if(BoilerPower<0){
		BoilerPower=0;
	}else if(BoilerPower>10){
		BoilerPower=10;
	}

	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_P, fround(P,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_I, fround(I,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPID_D, fround(D,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerPower, fround(BoilerPower,1));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BoilerTargetTempGraph, fround(BoilerTargetTemp,1));
}

void BoilerPWMTimerEvent(){ //PWM = ON at the beginning, OFF at the end of cycle
	long millisFromStart = millis()-BoilerPWMCycleStart;
		
	if( millisFromStart >= BoilerPeriodMillis ){ //time to START:
		if( !ErrorTempBoiler && PIDSTATUS>0 ){
			if( !ErrorTempHome && PIDSTATUS>1 ){  //vpin36 //0 1 2  manual BoilerPower / auto BoilerTemp / auto HomeTemp
				HomePIDEvaluation(); //evaluate BoilerTargetTemp
			}
			BoilerPIDEvaluation(); //once: on boiler pwm cycle start and only if temperature is really read
		}
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
  TempDS_Home.reset();
  TempDS_Home.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_Home.write(0x44); // start conversion
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
void MainCycle_ReadTempAndMoveValveEvent(); //declaration

void MainCycle_StartEvent() {

	switch(boardSTATUS){
		case Status_Standby:
			return;	//skip standby state
	}

	TempDS_AllStartConvertion();
	#ifdef testmode
	Serial.println("Start coversion... ");
	#endif

	timer.setTimeout(1000L, MainCycle_ReadTempAndMoveValveEvent); //start once after timeout
}

void MainCycle_ReadTempAndMoveValveEvent() {

  ErrorTempBoiler=0;
	ErrorTempFloorIn=0;
	ErrorTempFloorOut=0;
	ErrorTempHome=0;

	ErrorMeasTemp=0;
	
	if( !TempDS_GetTemp(&TempDS_Boiler,"BOILER",&tempBoiler) ){
	 ErrorMeasTemp++; ErrorTempBoiler++;
	}else
	  tempBoiler += offsetBoiler;
  
	if( !TempDS_GetTemp(&TempDS_FloorIn,"FLOORIN",&tempFloorIn) ){
	 ErrorMeasTemp++; ErrorTempFloorIn++;
	}else 
	  tempFloorIn += offsetFloorIn;
  
	if( !TempDS_GetTemp(&TempDS_FloorOut,"FLOOROUT",&tempFloorOut) ){
	 ErrorMeasTemp++; ErrorTempFloorOut++;
	}else 
	  tempFloorOut += offsetFloorOut;
	
	if( !TempDS_GetTemp(&TempDS_Home,"HOME",&tempHome) ){
	 ErrorMeasTemp++; ErrorTempHome++;
	}else
	  tempHome += offsetHome;

	#ifdef testmode
    Serial.println();
	#endif
  
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_Boiler,fround(tempBoiler,1)); //rounded 0.0 value
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_FloorIn,fround(tempFloorIn,1)); //rounded 0.0 value
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_FloorOut,fround(tempFloorOut,1)); //rounded 0.0 value
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_Home,fround(tempHome,1)); //rounded 0.0 value

  if( !ErrorMeasTemp && VALVESTATUS == 1 ){ //vpin37 = 0 1    manual / auto floorIn
	  float needChangeInTemp=0, needChangeSeconds=0;

		needChangeInTemp = tempTargetFloorIn - tempFloorIn;
		needChangeSeconds = needChangeInTemp*3;
		
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

		  ValveStartDecrease();
			timer.setTimeout(needChangeMillis, ValveStop);

		}else if(needChangeSeconds > 0){

		  needChangeMillis = needChangeSeconds * (float)Valve_OneMoveStepMillis;

		  //addCANMessage2Queue(VPIN_MotorValvePlus,needChangeMillis);

			ValveStartIncrease();
			timer.setTimeout(needChangeMillis, ValveStop);
		}
	}
	//addCANMessage2Queue(VPIN_BoilerPower,fround(BoilerPower,2); //rounded 0.00 value
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
	switch(vPinNumber){
		case VPIN_STATUS:
			boardSTATUS = (int)vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_PIDSTATUS:
			PIDSTATUS = (int)vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_VALVESTATUS:
			VALVESTATUS = (int)vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_SetMainCycleInterval:
			if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
				break;
			MainCycleInterval = (int)vPinValueFloat;
			timer.deleteTimer(mainTimerId);
			mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
			EEPROM_storeValues();
			break;
		case VPIN_ManualFloorIn:
			tempTargetFloorIn = vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_tempTargetFloorOut:
			tempTargetFloorOut = vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_ManualMotorValveMinus:
			if(vPinValueFloat==0) ValveStop(); else ValveStartDecrease();
			break;
		case VPIN_ManualMotorValvePlus:
			if(vPinValueFloat==0) ValveStop(); else ValveStartIncrease();
			break;
		case VPIN_SetBoilerPowerPeriodMinutes:
			if(vPinValueFloat<1) 
				vPinValueFloat=1;
			BoilerPowerPeriodMinutes = vPinValueFloat;
			BoilerPeriodMillis = (long)BoilerPowerPeriodMinutes*60000L;
			EEPROM_storeValues();
			break;
		case VPIN_BoilerPID_Kp: BoilerPID_Kp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_BoilerPID_Ki: BoilerPID_Ki = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_BoilerPID_Kd: BoilerPID_Kd = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_BoilerPower:  BoilerPower  = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_BoilerTargetTemp: BoilerTargetTemp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_SetBoilerPID_Isum_Zero: BoilerPID_Isum=0; break;

		case VPIN_HomePID_Kp: HomePID_Kp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_HomePID_Ki: HomePID_Ki = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_HomePID_Kd: HomePID_Kd = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_HomeTargetTemp: HomeTargetTemp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_SetHomePID_Isum_Zero: HomePID_Isum=0; break;
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

void EEPROM_storeValues(){
  EEPROM.put(eepromVIAddr,eepromValueIs);
  
	EEPROM.put(VPIN_STATUS*sizeof(float),						boardSTATUS);
  EEPROM.put(VPIN_MainCycleInterval*sizeof(float),MainCycleInterval);

  EEPROM.put(VPIN_ManualFloorIn*sizeof(float), 			tempTargetFloorIn);
  EEPROM.put(VPIN_tempTargetFloorOut*sizeof(float), tempTargetFloorOut);
  EEPROM.put(VPIN_SetBoilerPowerPeriodMinutes*sizeof(float), BoilerPowerPeriodMinutes);
  EEPROM.put(VPIN_BoilerPID_Kp*sizeof(float), 		BoilerPID_Kp);
  EEPROM.put(VPIN_BoilerPID_Ki*sizeof(float), 		BoilerPID_Ki);
  EEPROM.put(VPIN_BoilerPID_Kd*sizeof(float), 		BoilerPID_Kd);
  EEPROM.put(VPIN_BoilerPower*sizeof(float), 			BoilerPower);
  EEPROM.put(VPIN_BoilerTargetTemp*sizeof(float), BoilerTargetTemp);

	EEPROM.put(VPIN_HomeTargetTemp*sizeof(float), HomeTargetTemp);
  EEPROM.put(VPIN_HomePID_Kp*sizeof(float), 		HomePID_Kp);
  EEPROM.put(VPIN_HomePID_Ki*sizeof(float), 		HomePID_Ki);
  EEPROM.put(VPIN_HomePID_Kd*sizeof(float), 		HomePID_Kd);

	EEPROM.put(VPIN_PIDSTATUS*sizeof(float),			PIDSTATUS);
  EEPROM.put(VPIN_VALVESTATUS*sizeof(float),		VALVESTATUS);
  
}
void EEPROM_restoreValues(){
  int ival;
  EEPROM.get(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }
	
	EEPROM.get(VPIN_STATUS*sizeof(float),boardSTATUS);
  int aNewInterval;
	EEPROM.get(VPIN_MainCycleInterval*sizeof(float),aNewInterval);
  if(aNewInterval > 0){
    MainCycleInterval = aNewInterval;
  }
  
  EEPROM.get(VPIN_ManualFloorIn*sizeof(float), 			tempTargetFloorIn);
  EEPROM.get(VPIN_tempTargetFloorOut*sizeof(float), 	tempTargetFloorOut);
  EEPROM.get(VPIN_SetBoilerPowerPeriodMinutes*sizeof(float), BoilerPowerPeriodMinutes);
  EEPROM.get(VPIN_BoilerPID_Kp*sizeof(float), 			BoilerPID_Kp);
  EEPROM.get(VPIN_BoilerPID_Ki*sizeof(float), 			BoilerPID_Ki);
	EEPROM.get(VPIN_BoilerPID_Kd*sizeof(float), 			BoilerPID_Kd);
  EEPROM.get(VPIN_BoilerPower*sizeof(float), 			BoilerPower);
  EEPROM.get(VPIN_BoilerTargetTemp*sizeof(float), 	BoilerTargetTemp);

	EEPROM.get(VPIN_HomeTargetTemp*sizeof(float), 		HomeTargetTemp);
	EEPROM.get(VPIN_HomePID_Kp*sizeof(float), 				HomePID_Kp);
  EEPROM.get(VPIN_HomePID_Ki*sizeof(float), 				HomePID_Ki);
	EEPROM.get(VPIN_HomePID_Kd*sizeof(float), 				HomePID_Kd);

	EEPROM.get(VPIN_PIDSTATUS*sizeof(float), 				PIDSTATUS);
	EEPROM.get(VPIN_VALVESTATUS*sizeof(float), 			VALVESTATUS);
}
////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
	boardSTATUS = Status_Manual; //init
	EEPROM_restoreValues();

  #ifdef testmode
	Serial.begin(115200);
	#endif

  pinMode(MAX6675_CS,OUTPUT);
  digitalWrite(MAX6675_CS, HIGH); //turn off thermocouple CS
  KTCtimerID = timer.setInterval(1000L * 2, KTCtimer_StartEvent); //start regularly 
	
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

	pinMode(BOILER_ON_PIN,OUTPUT);
  digitalWrite(BOILER_ON_PIN,LOW); //turn off BOILER
	
  //turn off relays:
  pinMode(ValveMinusPin,OUTPUT);
  pinMode(ValvePlusPin,OUTPUT);
  ValveStop(); //initial

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
		#ifdef testmode
		Serial.println("Error Initializing CAN bus driver MCP2515...");
		#endif
	}

  //initialize filters Masks(0-1),Filters(0-5):
  // unsigned long mask  = (0x0100L | CAN_Unit_MASK | CAN_MSG_MASK)<<16;			//0x0F	0x010F0000;
  // unsigned long filt0 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_UNITCMD)<<16;	//0x04	0x01040000;
  // unsigned long filt1 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_INF)<<16;	//0x04	0x01040000;
  // CAN0.init_Mask(0,0,mask);                // Init first mask...
  // CAN0.init_Filt(0,0,filt0);                // Init first filter...
  // #ifdef testmode
  // CAN0.init_Filt(1,0,filt1);                // Init second filter...
  // #endif
  //CAN0.init_Mask(1,0,0x01FFFFFF);                // Init second mask...
  //CAN0.init_Filt(2,0,0x01FFFFFF);                // Init third filter...
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
