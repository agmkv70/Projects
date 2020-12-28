// ///VentTEH///

#include <OneWire.h>
#include <SimpleDHT.h>

//#define testmode
//#define testmodeCAN

//K-thermocouple pins:
#define MAX6675_CS PIN_A4
#define MAX6675_SO     12
#define MAX6675_SCK    13
#define TempIn_DHT_PIN  6
#define TempOut_DS_PIN  7

#define CAN_PIN_INT   9    
#define CAN_PIN_CS   10 
#include <NIK_defs.h>
#include <NIK_can.h>

#define LED_PIN 13
#define ValveOpen_PIN  PIN_A2 //ssr low current
#define ValveClose_PIN PIN_A3 //ssr low current
#define TEH_SSR_PIN         3 //TEH ssr high current relay on pin
#define PROTECTION_READ_PIN 4 //read heater protection (bimetal mechanical thermorelays are on or off)
#define PROTECTION_ON_PIN   5 //turn on protection relay

//byte space[100];
int targetHeaterStatus =0; //off+close=0,1//off+open=2//fan+open=3//fan+heat(kPwr)=4//fan+heat(PID)=5
    //currentHeaterStatus=0, //0(off+closed), 1(opening), 2(opening+heating), 3(opened),
                             //4(opened+heating), 5(blowing(cooling)), 6(closing), 10(ERROR)
int errorThermocouple=0;     //TEH overheat by thermocouple, thermocouple not giving data
int errorTEHOverheatError=0,curErrorRelayProtect=0; //1(can't turn ON), 2(can't turn OFF) //TEH overheat by relay protection
int VALVESTATUS=0;   //0 1 (closed/opened), 2(opening), 3(closing)
int valveStopTimerId=-1, valveCloseTimerId=-1;
static unsigned long ValveMovementTimeMil=15000;
int TEHPIDSTATUS=0;   //0 1 2 (off/onPID/onKPWR/error)
int PROTECTIONRELAYSTATUS=0; //to turn on this relay (with FAN and TEH on it: FAN as a way to manage it, TEH for protection) 
							 //while just blowing without heating (maybe its testing mode only and later I'll manage FAN in other way)
    


float TEHPower=0,TEHPower_PIDCorrection=0; //TEH power on time factor 0.0 .. 10.0 float
int   TEHPowerCurrentStateOnOff=0; //0=on or 1=off
static int   TEHPowerPeriodSeconds=5; //period of PWM in sec
static unsigned long  TEHPeriodMillis=2000L; //period of PWM in millis
unsigned long  TEHPWMCycleStart=0; //last cycle start
float TEHPID_KCoef=0.0001, //for convenience multiply all TEHPID_K
      TEHPID_Kp=5, TEHPID_Ki=1, TEHPID_Kd=7;
float TEHPID_Isum=0, TEHPID_prevtempTEH=0, TEHPID_prevtempAirOut=0;

float KdT_TEH=2, minKdT_TEH=1, maxKdT_TEH=10; //coef: TEHTargetTemp = tempAirIn + KdT_TEH * (AirOutTargetTemp-tempAirIn)
float kPwr2Air=0.24; //kPwr mode: How much power(0..10) needed to heat flowing air for 1*C
float kPwr_preMillisPerC=3000; //kPwr mode: full power preheat millis/*C
int kPwr_preheatStart=0,kPwr_PreheatIsOn=0; //kPwr mode: flag to start preheating
unsigned long kPwr_lastPreheatStartMillis=0, minpreheatRepeatPeriodMillis=5*60*1000L, StopPreheatTimerId=-1; //remember for not to preheat too often

float AirOutTargetTemp=20;
static float AirOutTargetTemp_MIN=15;
static float AirOutTargetTemp_MAX=28;//30-test, 24-real
float TEHTargetTemp=40;
static float TEHTargetTemp_MIN=18;
static float TEHTargetTemp_MAX=150;//100-test

static float TEHMaxTemp=220;  //защита; надо смотреть какой максимум выставить (по идее надо динамически с учетом внешней темп.)
//static float TEHMaxTempIncreasePerControlPeriod=100, TEHIncreaseControlPeriodSec=10;
//надо двойную защиту: по абс.макс. и по скорости прироста температуры выставить:
//- если за заданное время прирост больше максимума - значит нет продува!

float tempAirIn=0, humidityAirIn=NAN, tempTEH=0;
float tempAirOut=20, offsetAirOut=0; //температура и калибровочное смещение(если знаю)
int   ErrorTempAirIn=0,ErrorHumidityAirIn=0,
      ErrorTempTEH=0,ErrorTempAirOut=0;
unsigned long millisLastReport=0; //not too often report in CommandCycle

SimpleDHT22 dht_AirIn(TempIn_DHT_PIN);
OneWire  TempDS_AirOut(TempOut_DS_PIN); 

#ifdef testmode
int ReadTempCycleInterval=5; //часто - отадка 10 сек
#endif
#ifndef testmode
int ReadTempCycleInterval=5; //изредка 60 сек
#endif
int eepromVIAddr=1000,eepromValueIs=7730+5; //if this is in eeprom, then we got valid values, not junk

int readTempTimerId=-1, TEHPWMTimerId=-1, KTCtimerId=-1, commandTimerId=-1;

void StopPreheat(){
  kPwr_PreheatIsOn=0;
  StopPreheatTimerId=-1;
}
///////////////////////////////////TEH kPwr//////////////////////////////
void TEH_kPwr_Evaluation(){
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEH_kPwrPreheatON, kPwr_PreheatIsOn);
  if(ErrorTempAirIn!=0){
    TEHPower=0;
    return;
  }

  if(kPwr_PreheatIsOn==1){
    TEHPower=10;
    return;
  }

  if(kPwr_preheatStart==1){ //start preheating:
    kPwr_preheatStart=0;
    if(AirOutTargetTemp-tempAirOut<0){
      addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEH_ErrType, 101);
      return;
    }
    unsigned long preheatMillis = (float)(kPwr_preMillisPerC) * (AirOutTargetTemp-tempAirOut);
    if(preheatMillis==0 //nothing to start
      || (kPwr_lastPreheatStartMillis!=0 && millis()-kPwr_lastPreheatStartMillis < minpreheatRepeatPeriodMillis )){ //its too often
      addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEH_ErrType, 102);
      return; //no start
    }

    kPwr_lastPreheatStartMillis = millis();
    kPwr_PreheatIsOn = 1;
    StopPreheatTimerId = timer.setTimeout(preheatMillis, StopPreheat); //start once after timeout
    TEHPower=10;
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEH_ErrType, preheatMillis);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEH_kPwrPreheatON, kPwr_PreheatIsOn);
    return;
  }
  
  //regular kPwr regime:
  TEHPower = kPwr2Air * (AirOutTargetTemp-tempAirIn);
  if(TEHPower<0){
    TEHPower=0;
  }else if(TEHPower>10){
    TEHPower=10;
  }
  
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower,1));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirOutTargetTemp, fround(AirOutTargetTemp,1));
}

///////////////////////////////////TEH PID//////////////////////////////
void TEHPIDCorrectionEvaluation(){ //calc TEHPower_PIDCorrection  - в пределах +-3
  if(kPwr_PreheatIsOn==1){
    TEHPower_PIDCorrection=0;
    return;
  }

  float delta = (AirOutTargetTemp - tempAirOut)*TEHPID_KCoef;
  
  if(TEHPID_prevtempAirOut==0)
    TEHPID_prevtempAirOut = tempAirOut; //init TEHPID_prevtempTEH
    
  //sum Integral part of PID only if our result is not saturated:
  if(TEHPower+TEHPower_PIDCorrection<=0 && delta<0)
    ; //don't go too far down
  else{ 
    if(TEHPower+TEHPower_PIDCorrection>=10 && delta>0)
      TEHPID_Isum = 0; //don't go too far up & fallback Isum (not to become too hot)
    else
      TEHPID_Isum = TEHPID_Isum + delta;
  }
  if(TEHPID_Isum<0 && delta>0){//when coming over zero delta - nullify intergal part
    TEHPID_Isum=0;
  }else if(TEHPID_Isum>0 && delta<0){
    TEHPID_Isum=0;
  }
  //restrict Isum:
  if(TEHPID_Isum<-2){
    TEHPID_Isum=-2;
  }else if(TEHPID_Isum>2){
    TEHPID_Isum=2;
  }
  
  float P = TEHPID_Kp * delta;
  float I = TEHPID_Ki * TEHPID_Isum;
  float D = TEHPID_Kd * ((TEHPID_prevtempAirOut - tempAirOut)*TEHPID_KCoef); //new D calculation (absolute) - stable when target changes
  TEHPID_prevtempAirOut = tempAirOut;

  TEHPower_PIDCorrection = TEHPower_PIDCorrection + P + I + D;
  
  if(TEHPower_PIDCorrection<-3){
    TEHPID_Isum = 0;
    TEHPower_PIDCorrection=-3;
  }else if(TEHPower_PIDCorrection>3){
    TEHPID_Isum = 0;
    TEHPower_PIDCorrection=3;
  }
  if(TEHPower+TEHPower_PIDCorrection<0){
    TEHPower_PIDCorrection = TEHPower_PIDCorrection - (TEHPower+TEHPower_PIDCorrection);
  }else if(TEHPower+TEHPower_PIDCorrection>10){
    TEHPower_PIDCorrection = TEHPower_PIDCorrection - (TEHPower+TEHPower_PIDCorrection-10);
  }
  
  //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_P, fround(P,2));
  //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_I, fround(I,2));
  //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_D, fround(D,2));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower+TEHPower_PIDCorrection,1));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower_PIDCorrection, fround(TEHPower_PIDCorrection,1));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirOutTargetTemp, fround(AirOutTargetTemp,1));
}

void TEHPWMTimerEvent(){ //PWM = ON at the beginning, OFF at the end of cycle
  if(TEHPIDSTATUS==0){
    digitalWrite(TEH_SSR_PIN,LOW);
    return;
  }
  if(ErrorTempAirIn || ErrorTempTEH || ErrorTempAirOut){
    digitalWrite(TEH_SSR_PIN,LOW);
    return;
  }

  unsigned long millisFromStart = millis()-TEHPWMCycleStart;
    
  if( millisFromStart >= TEHPeriodMillis ){ //time to START:
    if( !ErrorTempTEH ){

      if( TEHPIDSTATUS==1 ){
        TEH_kPwr_Evaluation();
        //calc correction to kPwr:
        TEHPIDCorrectionEvaluation(); //once: on TEH pwm cycle start and only if temperature is really read

      }else if( TEHPIDSTATUS==2 ){
        TEH_kPwr_Evaluation();
        TEHPower_PIDCorrection=0;
      }
    }
    TEHPWMCycleStart = millis();
    millisFromStart=0;
  }

  //evaluate millisOn (so to say, transformation of TEHPower):
  unsigned long millisOn = ((TEHPower+TEHPower_PIDCorrection)/10*(float)TEHPeriodMillis); //sould be on in cycle
  if((TEHPower+TEHPower_PIDCorrection)<0.1f) millisOn = 0;
  else if((TEHPower+TEHPower_PIDCorrection)>9.9f) millisOn = TEHPeriodMillis;

  if(TEHPowerCurrentStateOnOff==0){ //we are off - we can only turn on:
    if( millisFromStart < millisOn ){ //time to START:
      if(millisOn > 0) TEHPowerCurrentStateOnOff=1;
      //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPowerOnOff, TEHPowerCurrentStateOnOff);
    }
  }else{ //TEHPowerCurrentStateOnOff==1 //we are ON - we can only turn off:
      if( millisFromStart >= millisOn ){ //time to TURN OFF:
        if(millisOn < TEHPeriodMillis)  TEHPowerCurrentStateOnOff=0;
        //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPowerOnOff, TEHPowerCurrentStateOnOff);
      }
  }

  if(tempAirOut > AirOutTargetTemp_MAX){ //additional protection
    TEHPowerCurrentStateOnOff=0;
  }
  if(tempTEH > TEHMaxTemp){ //additional protection
    TEHPowerCurrentStateOnOff=0;
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
  Serial.println(*temp);
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

void ReadTemperatureCycle_ReadTempEvent(); //declaration
void ReadTemperatureCycle_StartEvent() {

  switch(boardSTATUS){
    case Status_Standby:
      return;  //skip standby state
  }

  TempDS_AllStartConvertion();
  #ifdef testmode
  Serial.println("Run DS coversion... ");
  #endif
  
  timer.setTimeout(1000L, ReadTemperatureCycle_ReadTempEvent); //start once after timeout 1s
} //wait 1 sec and run next function:
void ReadTemperatureCycle_ReadTempEvent() {

  ErrorTempAirIn=0;
  ErrorHumidityAirIn=0;
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
  
  //if(ErrorTempTEH==0){
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHTemp,fround(tempTEH,0)); //rounded 0.0 value
  //}
}

void KTCReadThermocouple_Event(){
  ErrorTempTEH=0;

  tempTEH = readThermocoupleMAX6675();
  if(tempTEH==NAN){
    ErrorTempTEH++;
    tempTEH = -39.9;
  }
  if(tempTEH<-40){
    ErrorTempTEH++;
    tempTEH = -40;
  }
  if(tempTEH>500){
    ErrorTempTEH++;
    tempTEH = 500;
  }
  //send temp with other temps, but not here
  //else{
    //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHTemp,fround(tempTEH,0)); //rounded 0.0 value
  //}

  #ifdef testmode
  //Serial.println();
  Serial.print("TEH: KTC_MAX6675 = ");
  Serial.print(fround(tempTEH,1));
  Serial.println();
  #endif
}

void ValveStop(){
  digitalWrite(ValveOpen_PIN,LOW);
  digitalWrite(ValveClose_PIN,LOW);
  if(valveStopTimerId !=-1 ){ 
    timer.deleteTimer(valveStopTimerId);
    valveStopTimerId = -1;
  }
  if(VALVESTATUS==2){//opening
    VALVESTATUS=1;//opened
  }else if(VALVESTATUS==3){//closing
    VALVESTATUS=0;//closed
  }
}
void ValveClose(){
  //addCANMessage2QueueStr( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL, "v_close");
  PROTECTIONRELAYSTATUS=0; //turn off fan if it is connected to protection relay
  if(valveStopTimerId !=-1 ){ //clear old timer and start a new one
    timer.deleteTimer(valveStopTimerId);
    valveStopTimerId = -1;
  }
  if(valveCloseTimerId !=-1 ){
    timer.deleteTimer(valveCloseTimerId);
    valveCloseTimerId = -1;
  }
  digitalWrite(ValveOpen_PIN,LOW);
  delay(20);
  digitalWrite(ValveClose_PIN,HIGH);
  VALVESTATUS=3;//closing
  valveStopTimerId = timer.setTimeout(ValveMovementTimeMil, ValveStop); //start once after timeout
}
void ValveOpen(){
  //addCANMessage2QueueStr( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL, "v_open");
  if(valveStopTimerId !=-1 ){ //clear old timer and start a new one
    timer.deleteTimer(valveStopTimerId);
    valveStopTimerId = -1;
  }
  if(valveCloseTimerId !=-1 ){
    timer.deleteTimer(valveCloseTimerId);
    valveCloseTimerId = -1;
  }
  digitalWrite(ValveClose_PIN,LOW);
  delay(20);
  digitalWrite(ValveOpen_PIN,HIGH);
  VALVESTATUS=2;//opening
  valveStopTimerId = timer.setTimeout(ValveMovementTimeMil, ValveStop); //start once after timeout
}

int freeRAM(){
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start: (int) __brkval);
}

void onChangeHeaterStatus(int old_targetHeaterStatus){
}

void InsureSafeValues(){
  if(AirOutTargetTemp<AirOutTargetTemp_MIN) AirOutTargetTemp=AirOutTargetTemp_MIN;
  if(AirOutTargetTemp>AirOutTargetTemp_MAX) AirOutTargetTemp=AirOutTargetTemp_MAX;
  if(TEHTargetTemp<TEHTargetTemp_MIN) TEHTargetTemp=TEHTargetTemp_MIN;
  if(TEHTargetTemp>TEHTargetTemp_MAX) TEHTargetTemp=TEHTargetTemp_MAX;
}
//////////////////////COMMANDer////////////////////// STATUS changes:
void CommandCycle_Event(){
  InsureSafeValues();
  //Serial.println(targetHeaterStatus);
  //#ifdef testmode
  if(millis()-millisLastReport > 10000L){
    millisLastReport = millis();
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HEATER_TEHPIDSTATUS, TEHPIDSTATUS);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround((TEHPIDSTATUS>0 ? TEHPower : 0), 1));
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HEATER_VALVESTATUS, VALVESTATUS);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HEATER_TEHERROR, errorTEHOverheatError);
    //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HEATER_PIN4READ, digitalRead(PROTECTION_READ_PIN));
    //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_HEATER_FREERAM, fround(freeRAM(),0));
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL, fround(millis()/1000,0));
  }
  //#endif

  if(digitalRead(PROTECTION_READ_PIN)==LOW){
    if(TEHPIDSTATUS==0 && PROTECTIONRELAYSTATUS==0){ //relay error2 (can't turn it off)
      curErrorRelayProtect=2;
      errorTEHOverheatError = curErrorRelayProtect; //wait manual command to restart
    }
  }else{//PROTECTION_READ_PIN==HIGH
    if(TEHPIDSTATUS==1 || PROTECTIONRELAYSTATUS==1){ //protection error1 (probably overheat!) (can't turn on because protection has broken circuit)
      curErrorRelayProtect=1;
      errorTEHOverheatError = curErrorRelayProtect; //wait manual command to restart
    }
  }
  errorTEHOverheatError=0;//testing!!!!!!!!!!!!!!!!!!!!!!!!!!****************************
  curErrorRelayProtect=0; //testing!!!!!!!!!!!!!!!!!!!!!!!!!!****************************
  if(curErrorRelayProtect!=0 || errorTEHOverheatError!=0){
    TEHPIDSTATUS=0;
	  PROTECTIONRELAYSTATUS=0;
  }
  curErrorRelayProtect=0; 

  switch(targetHeaterStatus){

    case 0: //closed+off
    case 1: //closed+off
      TEHPIDSTATUS=0; //turn off TEH
	    if(VALVESTATUS==0 || VALVESTATUS==3){ //0 closed,3 closing
	  	  ;//nothing
      }else{
        if(ErrorTempTEH!=0){//cant read TEH temperature
          //close after 20 sec. - to cool down a bit:
          VALVESTATUS=3; //closing
          valveCloseTimerId = timer.setTimeout(20*1000L, ValveClose);
        }else{//read KTC and decide weather TEH is cooled enough:
          if(tempTEH!=NAN && tempTEH>50){
            //if(valveCloseTimerId !=0 ){//no error now, clear scheduled closing:
            //  timer.deleteTimer(valveCloseTimerId);
            //}
            ;//its too hot - just wait
          }else{ //now its cold:
            ValveClose();
          }
        }
      }
    break;

    case 2: //opened+off
      TEHPIDSTATUS=0; //turn off TEH
      if(VALVESTATUS==1 || VALVESTATUS==2){ //1 opened,2 opening
        if(tempTEH!=NAN && tempTEH>50){
          ;//its too hot - just wait
        }else{ //now its cold:
          PROTECTIONRELAYSTATUS=0; //turn off FAN
        }
      }else{
        ValveOpen();
      }
    break;

    case 3: //open+FAN (without heater)
      TEHPIDSTATUS=0; //turn off TEH
      if(VALVESTATUS==1 || VALVESTATUS==2){ //1 opened,2 opening
        if(VALVESTATUS==1 && errorTEHOverheatError==0){ //1 opened
          PROTECTIONRELAYSTATUS=1; //turn on FAN (if its on same relay as protection)
        }
      }else{
        ValveOpen();
      }
    break;

    case 4: //open+fan+heat(kPwr)
      
      if(VALVESTATUS==1 || VALVESTATUS==2){ //0 opened,2 opening
	  	  if(VALVESTATUS==1 && errorTEHOverheatError==0){ //1 opened
          if(TEHPIDSTATUS==0){ //it was off
            TEHPID_prevtempTEH=0; //init for no PID start lags
            kPwr_preheatStart=1;
          }
          TEHPIDSTATUS=2; //turn on TEH
          PROTECTIONRELAYSTATUS=1; //turn on FAN (if its on same relay as protection)
        }
      }else{//0-closed,3-closing
        ValveOpen();
        //if(ErrorTempTEH==0 && tempTEH<40.0){
        //  TEHPower = 1; //its cold - quick start
        //}else{
        //  TEHPower=0;  //its hot or not measured - careful start
        //}
      }
    break;

    case 5: //open+fan+heat(PID)
      if(VALVESTATUS==1 || VALVESTATUS==2){ //0 opened,2 opening
	  	  if(VALVESTATUS==1 && errorTEHOverheatError==0){ //1 opened
          if(TEHPIDSTATUS==0){ //its turning on
            TEHPID_prevtempTEH=0; //init for no PID start lags
		      }
          TEHPIDSTATUS=1; //turn on TEH
          PROTECTIONRELAYSTATUS=1; //turn on FAN (if its on same relay as protection)
        }
      }else{
        ValveOpen();
        if(ErrorTempTEH==0 && tempTEH<40.0){
          TEHPower = 1; //its cold - quick start
        }else{
          TEHPower=0;  //its hot or not measured - careful start
        }
      }
    break;
  }

  if((TEHPIDSTATUS==0 && PROTECTIONRELAYSTATUS==0) || errorTEHOverheatError!=0){
    digitalWrite(PROTECTION_ON_PIN, LOW);
    TEHPID_Isum=0; //clear integral part of PID
  }else{
	  if(errorTEHOverheatError==0){
      digitalWrite(PROTECTION_ON_PIN, HIGH);
	  }
  }//till next run time - relay will have time to switch
}

//////////////////////CAN commands///////////////////
char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
  // #ifdef testmode
  // Serial.print("received CAN message: VPIN=");
  // Serial.print(vPinNumber);
  // Serial.print(" FloatValue=");
  // Serial.print(vPinValueFloat);
  // Serial.println();
  // #endif
  switch(vPinNumber){
    case VPIN_HEATER_TRGSTATUS:{
      addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL, vPinNumber*100+(int)vPinValueFloat); //just pin number and value coded
      int old_targetHeaterStatus=targetHeaterStatus;
      targetHeaterStatus = (int)vPinValueFloat;
      EEPROM_storeValues();
      onChangeHeaterStatus(old_targetHeaterStatus);
      break;
    }
    case VPIN_ClearTEHOverheatError:
      errorTEHOverheatError = 0;
      break;
    case VPIN_TEH_KdTempAirIn:
      if(minKdT_TEH > vPinValueFloat)
        vPinValueFloat = minKdT_TEH;
      if(vPinValueFloat > maxKdT_TEH)
        vPinValueFloat = maxKdT_TEH;
      KdT_TEH = vPinValueFloat;
      EEPROM_storeValues();
      break;
    // case VPIN_HEATER_SetReadTempCycleInterval:
    //   if(ReadTempCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
    //     break;
    //   ReadTempCycleInterval = (int)vPinValueFloat;
    //   timer.deleteTimer(readTempTimerId);
    //   readTempTimerId = timer.setInterval(1000L * ReadTempCycleInterval, ReadTemperatureCycle_StartEvent); //start regularly
    //   EEPROM_storeValues();
    //   break;
    case VPIN_SetTEHPowerPeriodSeconds:
      if(vPinValueFloat<1) 
        vPinValueFloat=1;
      TEHPowerPeriodSeconds = vPinValueFloat;
      TEHPeriodMillis = (unsigned long)TEHPowerPeriodSeconds*1000L;
      EEPROM_storeValues();
      break;
    case VPIN_TEHPID_Kp: TEHPID_Kp = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_TEHPID_Ki: TEHPID_Ki = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_TEHPID_Kd: TEHPID_Kd = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_TEHPower:  TEHPower  = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_AirOutTargetTemp: AirOutTargetTemp = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_SetTEHPID_Isum_Zero: TEHPID_Isum=0; break;
    case VPIN_TEH_kPwr: kPwr2Air = vPinValueFloat; EEPROM_storeValues(); break;
    case VPIN_TEH_kPwr_preMillisPerC: kPwr_preMillisPerC = vPinValueFloat; EEPROM_storeValues(); break;
    
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
  InsureSafeValues();
  return;
  EEPROM.put(eepromVIAddr,eepromValueIs);
  
  //EEPROM.put(VPIN_STATUS*sizeof(float),            boardSTATUS);
  //EEPROM.put(VPIN_MainCycleInterval*sizeof(float),ReadTempCycleInterval);

  //EEPROM.put(VPIN_ManualFloorIn*sizeof(float),       tempTargetFloorIn);
  //EEPROM.put(VPIN_tempTargetFloorOut*sizeof(float), tempTargetFloorOut);
  EEPROM.put(VPIN_SetTEHPowerPeriodSeconds*sizeof(float), TEHPowerPeriodSeconds);
  EEPROM.put(VPIN_TEHPID_Kp*sizeof(float),     TEHPID_Kp);
  EEPROM.put(VPIN_TEHPID_Ki*sizeof(float),     TEHPID_Ki);
  EEPROM.put(VPIN_TEHPID_Kd*sizeof(float),     TEHPID_Kd);
  EEPROM.put(VPIN_TEHPower*sizeof(float),      TEHPower);
  EEPROM.put(VPIN_AirOutTargetTemp*sizeof(float), AirOutTargetTemp);
  EEPROM.put(VPIN_TEH_kPwr*sizeof(float),   kPwr2Air);
  EEPROM.put(VPIN_TEH_kPwr_preMillisPerC*sizeof(float),   kPwr_preMillisPerC);
  
  //EEPROM.put(VPIN_PIDSTATUS*sizeof(float),      TEHPIDSTATUS);
  //EEPROM.put(VPIN_VALVESTATUS*sizeof(float),    VALVESTATUS);
  
}
void EEPROM_restoreValues(){
  return;
  int ival;
  EEPROM.get(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }
  
  // EEPROM.get(VPIN_STATUS*sizeof(float),boardSTATUS);
  // int aNewInterval;
  // EEPROM.get(VPIN_MainCycleInterval*sizeof(float),aNewInterval);
  // if(aNewInterval > 0){
  //   ReadTempCycleInterval = aNewInterval;
  // }
  
  //EEPROM.get(VPIN_ManualFloorIn*sizeof(float),       tempTargetFloorIn);
  //EEPROM.get(VPIN_tempTargetFloorOut*sizeof(float),   tempTargetFloorOut);
  EEPROM.get(VPIN_SetTEHPowerPeriodSeconds*sizeof(float), TEHPowerPeriodSeconds);
  EEPROM.get(VPIN_TEHPID_Kp*sizeof(float),       TEHPID_Kp);
  EEPROM.get(VPIN_TEHPID_Ki*sizeof(float),       TEHPID_Ki);
  EEPROM.get(VPIN_TEHPID_Kd*sizeof(float),       TEHPID_Kd);
  EEPROM.get(VPIN_TEHPower*sizeof(float),       TEHPower);
  EEPROM.get(VPIN_AirOutTargetTemp*sizeof(float),   AirOutTargetTemp);
  EEPROM.get(VPIN_TEH_kPwr*sizeof(float),       kPwr2Air);
  EEPROM.get(VPIN_TEH_kPwr_preMillisPerC*sizeof(float),       kPwr_preMillisPerC);
  
  //EEPROM.get(VPIN_PIDSTATUS*sizeof(float),         TEHPIDSTATUS);
  //EEPROM.get(VPIN_VALVESTATUS*sizeof(float),       VALVESTATUS);
  InsureSafeValues();
}

////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
  delay(1000);
  boardSTATUS = Status_Manual; //init
  EEPROM_restoreValues();

  //space[0]=55;//to use

  #ifdef testmode
  Serial.begin(115200);
  #endif

  pinMode(MAX6675_CS,OUTPUT);
  digitalWrite(MAX6675_CS, HIGH); //turn off thermocouple CS
  KTCtimerId = timer.setInterval(1000L*2, KTCReadThermocouple_Event); //2sec
  //readTempTimerId = timer.setInterval(1000L * ReadTempCycleInterval, ReadTemperatureCycle_StartEvent); //start regularly
  //return;

  pinMode(PROTECTION_ON_PIN,OUTPUT);
  digitalWrite(PROTECTION_ON_PIN,LOW); //turn off protection relay
  pinMode(PROTECTION_READ_PIN,INPUT);
  
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

  pinMode(TEH_SSR_PIN,OUTPUT);
  digitalWrite(TEH_SSR_PIN,LOW); //turn off TEH
  
  //turn off relays:
  pinMode(ValveOpen_PIN,OUTPUT);
  pinMode(ValveClose_PIN,OUTPUT);
  digitalWrite(ValveOpen_PIN,LOW);//turn off
  digitalWrite(ValveClose_PIN,LOW);//turn off
  VALVESTATUS=1;//1=opened
  ValveClose(); //initial closing

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
    #ifdef testmode
    Serial.println("Error Initializing CAN bus driver MCP2515...");
    #endif
  }

  //initialize filters Masks(0-1),Filters(0-5):
  // unsigned long mask  = (0x0100L | CAN_Unit_MASK | CAN_MSG_MASK)<<16;      //0x0F  0x010F0000;
  // unsigned long filt0 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_UNITCMD)<<16;  //0x04  0x01040000;
  // unsigned long filt1 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_INF)<<16;  //0x04  0x01040000;
  //receive 0x100 messages:
  CAN0.init_Mask(0,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
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
  
  //#ifdef testmode
  //CAN0.setMode(MCP_LOOPBACK);
  //#endif
  //#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data
  //#endif
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input

  commandTimerId = timer.setInterval(1000L, CommandCycle_Event); //500 if not test
  delay(100); //for two timers not at once
  readTempTimerId = timer.setInterval(1000L * ReadTempCycleInterval, ReadTemperatureCycle_StartEvent); //start regularly
  delay(150); //for two timers not at once
  TEHPWMTimerId = timer.setInterval(50, TEHPWMTimerEvent); //1 sec pwm discretion
  TEHPWMCycleStart = millis();
}

////////////////////////////////////////////////LOOP////////////////////////////
void loop(void) {
  timer.run();
  checkReadCAN();
}
