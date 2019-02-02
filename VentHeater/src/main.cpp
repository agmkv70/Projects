// ///VentTEH///

#include <OneWire.h>
#include <SimpleDHT.h>

#define testmode

//K-thermocouple pins:
#define MAX6675_CS   PIN_A4
#define MAX6675_SO   12
#define MAX6675_SCK  13
#define TempIn_DHT_PIN   6
#define TempOut_DS_PIN   7

#define CAN_PIN_INT 9    
#define CAN_PIN_CS  10 
#include <NIK_defs.h>
#include <NIK_can.h>

#define LED_PIN 13
#define ValveOpen_PIN   PIN_A2 //ssr low current
#define ValveClose_PIN  PIN_A3 //ssr low current
#define TEH_SSR_PIN  3        //TEH ssr high current relay on pin
#define PROTECTION_READ_PIN 4 //read heater protection (bimetal mechanical thermorelays are on or off)
#define PROTECTION_ON_PIN   5 //turn on protection relay

int targetHeaterStatus=0, //off/on
		currentHeaterStatus=0, //0(off,standby),1(opening),2(opening+heating),
		                      //3(heating), 4(blowing(cooling)), 5(closing), 6(ERROR)
    VALVESTATUS=0, //0 1 (off/on)
		TEHPIDSTATUS=0,	 //0 1 (off/on)
		xx;


float TEHPower=0; //TEH power on time factor 0.0 .. 10.0 float
int   TEHPowerCurrentStateOnOff=0; //0=on or 1=off
int   TEHPowerPeriodSeconds=5; //period of PWM in sec
long  TEHPeriodMillis=5000L; //period of PWM in millis
long  TEHPWMCycleStart=0; //last cycle start
float TEHPID_KCoef=0.1, //for convenience multiply all TEHPID_K
	    TEHPID_Kp=6, TEHPID_Ki=0.2, TEHPID_Kd=9;
float TEHPID_Isum=0, TEHPID_prevDelta=0;

float AirOutTargetTemp=21;
float AirOutTargetTemp_MIN=18;
float AirOutTargetTemp_MAX=24;

float TEHMaxTemp=250;  //защита; надо смотреть какой максимум выставить (по идее надо динамически с учетом внешней темп.)
float TEHMaxTempIncreasePerControlPeriod=100, TEHIncreaseControlPeriodSec=10;
//надо двойную защиту: по абс.макс. и по скорости прироста температуры выставить:
//- если за заданное время прирост больше максимума - значит нет продува!

float tempAirIn=0, humidityAirIn=NAN, tempTEH=0;
float tempAirOut=20, offsetAirOut=0; //температура и калибровочное смещение(если знаю)
float tempTargetAirOut=20;
int   ErrorTempAirIn=0,ErrorHumidityAirIn=0,ErrorTempTEH=0,ErrorTempAirOut=0;

SimpleDHT22 dht_AirIn(TempIn_DHT_PIN);
OneWire  TempDS_AirOut(TempOut_DS_PIN); 

#ifdef testmode
int MainCycleInterval=5; //часто - отадка 10 сек
#endif
#ifndef testmode
int MainCycleInterval=5; //изредка 60 сек
#endif
int eepromVIAddr=1000,eepromValueIs=7730+1; //if this is in eeprom, then we got valid values, not junk

int mainTimerId, TEHPWMTimerId;

///////////////////////////////////TEH PID//////////////////////////////
void TEHPIDEvaluation(){
	float delta = AirOutTargetTemp - tempAirOut;
	
	//sum Integral part of PID only if our result is not saturated:
	if(TEHPower<=0 && delta<0)
		; //don't go too far down
	else if(TEHPower>=10 && delta>0)
		; //don't go too far up
	else
		TEHPID_Isum = TEHPID_Isum + delta;

	float P = (TEHPID_Kp * TEHPID_KCoef) * delta;
	float I = (TEHPID_Ki * TEHPID_KCoef) * TEHPID_Isum;
	float D = (TEHPID_Kd * TEHPID_KCoef) * (delta - TEHPID_prevDelta);
	TEHPID_prevDelta = delta;

  TEHPower = TEHPower + P + I + D;
	
	if(TEHPower<0){
		TEHPower=0;
	}else if(TEHPower>10){
		TEHPower=10;
	}

	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_P, fround(P,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_I, fround(I,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_D, fround(D,2));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower,1));
	addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirOutTargetTempGraph, fround(AirOutTargetTemp,1));
}

void TEHPWMTimerEvent(){ //PWM = ON at the beginning, OFF at the end of cycle
	long millisFromStart = millis()-TEHPWMCycleStart;
		
	if( millisFromStart >= TEHPeriodMillis ){ //time to START:
		if( !ErrorTempAirOut && TEHPIDSTATUS>0 ){
			TEHPIDEvaluation(); //once: on TEH pwm cycle start and only if temperature is really read
		}
	  TEHPWMCycleStart = millis();
		millisFromStart=0;
	}

	//evaluate millisOn (so to say, transformation of TEHPower):
	long millisOn = (TEHPower/10*(float)TEHPeriodMillis); //sould be on in cycle
	if(TEHPower<0.1f) millisOn = 0;
	else if(TEHPower>9.9f) millisOn = TEHPeriodMillis;

	if(TEHPowerCurrentStateOnOff==0){ //we are off - we can only turn on:
		if( millisFromStart < millisOn ){ //time to START:
			if(millisOn > 0) TEHPowerCurrentStateOnOff=1;
			//addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPowerOnOff, TEHPowerCurrentStateOnOff);
		}
	}else{ //TEHPowerCurrentStateOnOff==1 //we are ON - we can only turn off:
			if( millisFromStart >= millisOn ){ //time to TURN OFF:
				if(millisOn < TEHPeriodMillis)	TEHPowerCurrentStateOnOff=0;
				//addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPowerOnOff, TEHPowerCurrentStateOnOff);
			}
	}

  if(TEHPowerCurrentStateOnOff==1)
		digitalWrite(TEH_SSR_PIN,HIGH);
	else
		digitalWrite(TEH_SSR_PIN,LOW);
}

/////////////////////////////////general program////////////////////////
void TempDS_AllStartConvertion() {
  TempDS_AirOut.reset();
  TempDS_AirOut.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_AirOut.write(0x44); // start conversion
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
  Serial.print(" ");Serial.print(dname);
	Serial.print("=");
  Serial.print(*temp);
	#endif

  return 1; //OK
}

////////////////////////////////////////////////////////////////////////
//Heater K-thermocouple:
float readThermocoupleMAX6675() {
  uint16_t data=0;
  //pinMode(MAX6675_SO, INPUT);
  //pinMode(MAX6675_SCK, OUTPUT);
  
	digitalWrite(CAN_PIN_CS, HIGH);
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  /*// Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  data = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  data <<= 8;
  data |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);*/
	
	// read 16 bits, MSB first
  data |= SPI.transfer(0) << 8;
  data |= SPI.transfer(0) << 0;
  
  digitalWrite(MAX6675_CS, HIGH);
	SPI.endTransaction();
  digitalWrite(CAN_PIN_CS, LOW);
  delay(1);
	if(data & 0x4){ // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }
	// The lower three bits (0,1,2) are discarded status bits
  data >>= 3;
  // The remaining bits are the number of 0.25 degree (C) counts
  return (float)data*0.25; //returning float
}

void MainCycle_ReadTempEvent(); //declaration
void MainCycle_StartEvent() {

	switch(boardSTATUS){
		case Status_Standby:
			return;	//skip standby state
	}

	TempDS_AllStartConvertion();
	#ifdef testmode
	Serial.println("Run DS coversion... ");
	#endif
	
	timer.setTimeout(1000L, MainCycle_ReadTempEvent); //start once after timeout 1s
} //wait 1 sec and run next function:
void MainCycle_ReadTempEvent() {

  ErrorTempAirIn=0;
	ErrorHumidityAirIn=0;
	ErrorTempTEH=0;
	ErrorTempAirOut=0;
	
	float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  
	if ((err = dht_AirIn.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess){ //not faster than once in 2 sec
		#ifdef testmode
		Serial.print("Read DHT22 failed, err="); Serial.println(err);
		#endif
  	ErrorTempAirIn++;
		ErrorHumidityAirIn++;
	}else{
		tempAirIn = temperature;
		humidityAirIn = humidity;

		addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirInTemp,fround(tempAirIn,0)); //rounded 0.0 value
	  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HumidityAirIn,fround(humidityAirIn,0)); //rounded 0.0 value
	}
	#ifdef testmode
	Serial.print(" DHT22: T= ");
	Serial.print(fround(tempAirIn,1));
	Serial.print(" Hum= ");
	Serial.print(fround(humidityAirIn,1));
	Serial.println();
  #endif
  
	if( !TempDS_GetTemp(&TempDS_AirOut,"AIROUT",&tempAirOut) ){
	 ErrorTempAirOut++;
	 #ifdef testmode
		Serial.print("Read DS failed!"); Serial.println();
	 #endif
	}else{
		addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirOutTemp,fround(tempAirOut,1)); //rounded 0.0 value
	}
	// #ifdef testmode
	// Serial.print(" DS: T= ");
	// Serial.print(fround(tempAirOut,1));
	// Serial.println();
  // #endif

	tempTEH = readThermocoupleMAX6675();
	if(tempTEH==NAN){
		ErrorTempTEH++;
	}else{
		addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHTemp,fround(tempTEH,0)); //rounded 0.0 value
	}

	#ifdef testmode
	Serial.println();
	Serial.print("TEH: KTC_MAX6675 = ");
	Serial.print(fround(tempTEH,1));
	Serial.println();
  #endif
}

//////////////////////CAN commands///////////////////
char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
	switch(vPinNumber){
		case VPIN_STATUS:
			boardSTATUS = (int)vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_PIDSTATUS:
			TEHPIDSTATUS = (int)vPinValueFloat;
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
		//case VPIN_ManualMotorValveMinus:
		//	if(vPinValueFloat==0) ValveStop(); else ValveStartDecrease();
		//	break;
		//case VPIN_ManualMotorValvePlus:
		//	if(vPinValueFloat==0) ValveStop(); else ValveStartIncrease();
		//	break;
		case VPIN_SetTEHPowerPeriodSeconds:
			if(vPinValueFloat<1) 
				vPinValueFloat=1;
			TEHPowerPeriodSeconds = vPinValueFloat;
			TEHPeriodMillis = (long)TEHPowerPeriodSeconds*1000L;
			EEPROM_storeValues();
			break;
		case VPIN_TEHPID_Kp: TEHPID_Kp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_TEHPID_Ki: TEHPID_Ki = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_TEHPID_Kd: TEHPID_Kd = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_TEHPower:  TEHPower  = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_AirOutTargetTemp: AirOutTargetTemp = vPinValueFloat; EEPROM_storeValues(); break;
		case VPIN_SetTEHPID_Isum_Zero: TEHPID_Isum=0; break;

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

//////////////////////EEPROM/////////////////////////
void EEPROM_storeValues(){
  EEPROM.put(eepromVIAddr,eepromValueIs);
  
	EEPROM.put(VPIN_STATUS*sizeof(float),						boardSTATUS);
  EEPROM.put(VPIN_MainCycleInterval*sizeof(float),MainCycleInterval);

  //EEPROM.put(VPIN_ManualFloorIn*sizeof(float), 			tempTargetFloorIn);
  //EEPROM.put(VPIN_tempTargetFloorOut*sizeof(float), tempTargetFloorOut);
  EEPROM.put(VPIN_SetTEHPowerPeriodSeconds*sizeof(float), TEHPowerPeriodSeconds);
  EEPROM.put(VPIN_TEHPID_Kp*sizeof(float), 		TEHPID_Kp);
  EEPROM.put(VPIN_TEHPID_Ki*sizeof(float), 		TEHPID_Ki);
  EEPROM.put(VPIN_TEHPID_Kd*sizeof(float), 		TEHPID_Kd);
  EEPROM.put(VPIN_TEHPower*sizeof(float), 			TEHPower);
  EEPROM.put(VPIN_AirOutTargetTemp*sizeof(float), AirOutTargetTemp);

	EEPROM.put(VPIN_PIDSTATUS*sizeof(float),			TEHPIDSTATUS);
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
  
  //EEPROM.get(VPIN_ManualFloorIn*sizeof(float), 			tempTargetFloorIn);
  //EEPROM.get(VPIN_tempTargetFloorOut*sizeof(float), 	tempTargetFloorOut);
  EEPROM.get(VPIN_SetTEHPowerPeriodSeconds*sizeof(float), TEHPowerPeriodSeconds);
  EEPROM.get(VPIN_TEHPID_Kp*sizeof(float), 			TEHPID_Kp);
  EEPROM.get(VPIN_TEHPID_Ki*sizeof(float), 			TEHPID_Ki);
	EEPROM.get(VPIN_TEHPID_Kd*sizeof(float), 			TEHPID_Kd);
  EEPROM.get(VPIN_TEHPower*sizeof(float), 			TEHPower);
  EEPROM.get(VPIN_AirOutTargetTemp*sizeof(float), 	AirOutTargetTemp);

	EEPROM.get(VPIN_PIDSTATUS*sizeof(float), 				TEHPIDSTATUS);
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
  //KTCtimerID = timer.setInterval(1000L * 2, KTCtimer_StartEvent); //start regularly 

	//mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
	//return;

  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

	pinMode(TEH_SSR_PIN,OUTPUT);
  digitalWrite(TEH_SSR_PIN,LOW); //turn off TEH
	
  //turn off relays:
  pinMode(ValveOpen_PIN,OUTPUT);
  pinMode(ValveClose_PIN,OUTPUT);
  //ValveStop(); //initial

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
	TEHPWMTimerId = timer.setInterval(1000L, TEHPWMTimerEvent); //1 sec pwm discretion
	TEHPWMCycleStart = millis();
}

////////////////////////////////////////////////LOOP////////////////////////////
void loop(void) {
  timer.run();
  checkReadCAN();
}
