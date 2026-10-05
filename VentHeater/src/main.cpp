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
#define _MAX_FIXEDARRAY_DEFINED 16 //CAN queue: temps + PID reports can come at once
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
static int   TEHPowerPeriodSeconds=2; //period of PWM in sec
static unsigned long  TEHPeriodMillis=2000L; //period of PWM in millis
unsigned long  TEHPWMCycleStart=0; //last cycle start
//PID on air-out temperature, added to kPwr power: power(0..10) = TEHPower(kPwr) + P + I + D
float TEHPID_Kp=1,     //power per 1*C of error
      TEHPID_Ki=0.003, //power per 1*C*sec of error
      TEHPID_Kd=0;     //power per 1*C/sec of air-out temperature change
float TEHPID_I=0;      //integral part, power units
float TEHPID_prevtempAirOut=NAN; //NAN = PID (re)started
unsigned long TEHPID_prevMillis=0;
byte  newAirOutReading=0; //1 = tempAirOut was really read since last PID step

//PID autotune (relay method): power switches between base+-step around AirOutTargetTemp
int   AT_state=0;          //0=off, 1=running, 100=done, <0=aborted (VPIN_TEHPID_AutoTune gets 1+cycles while running)
float AT_step=3, AT_hyst=0.3, AT_high=0, AT_low=0;
byte  AT_relayHigh=0, AT_highSwitches=0, AT_cycles=0;
float AT_max=-100, AT_min=1000, AT_sumAmp=0;
unsigned long AT_lastHighSwitch=0, AT_lastSwitch=0, AT_start=0, AT_sumPeriod=0;
#define AT_CYCLES_NEEDED 3                 //measured full cycles (after one skipped transient cycle)
#define AT_MAX_HALFCYCLE_MS (40*60*1000UL) //no switch for 40 min - process doesn't oscillate
#define AT_MAX_TOTAL_MS (4*60*60*1000UL)

float KdT_TEH=2, minKdT_TEH=1, maxKdT_TEH=10; //coef: TEHTargetTemp = tempAirIn + KdT_TEH * (AirOutTargetTemp-tempAirIn)
float kPwr2Air=0.34; //kPwr mode: How much power(0..10) needed to heat flowing air for 1*C
float kPwr_preMillisPerC=4000; //kPwr mode: full power preheat millis/*C
int kPwr_preheatStart=0,kPwr_PreheatIsOn=0; //kPwr mode: flag to start preheating
unsigned long kPwr_lastPreheatStartMillis=0, minpreheatRepeatPeriodMillis=5*60*1000L, StopPreheatTimerId=-1; //remember for not to preheat too often

float AirOutTargetTemp=20;
static float AirOutTargetTemp_MIN=15;
static float AirOutTargetTemp_MAX=32;//30-test, 24-real //32-cause theres strange readings (ir emmitance?)
float TEHTargetTemp=40;
static float TEHTargetTemp_MIN=18;
static float TEHTargetTemp_MAX=230;//100-test

static float TEHMaxTemp=250;  //защита; надо смотреть какой максимум выставить (по идее надо динамически с учетом внешней темп.)
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
int eepromVIAddr=1000,eepromValueIs=7730+6; //if this is in eeprom, then we got valid values, not junk (+6: new PID units)

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

  if( TEHPIDSTATUS!=1 ){ //its PID - sent later, after correction evaluation
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower,1));
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_AirOutTargetTemp, fround(AirOutTargetTemp,1));
  }
}

///////////////////////////////////TEH PID//////////////////////////////
void EEPROM_storeValues();

void TEHPID_Reset(){ //on (re)start of PID heating: no old state, no D kick
  TEHPID_I=0;
  TEHPID_prevtempAirOut=NAN;
  TEHPower_PIDCorrection=0;
}

void AT_Report(){
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_AutoTune, AT_state==1 ? 1+AT_cycles : AT_state);
}
void AT_Abort(int reason){ //reason<0: -1 not in PID mode, -2 no oscillation, -3 too long, -4 oscillation too small
  if(AT_state!=1) return;
  AT_state=reason;
  TEHPID_Reset();
  AT_Report();
}
void AT_Start(float step){ //only in PID heating mode (targetHeaterStatus=5, valve opened, TEH on)
  if(TEHPIDSTATUS!=1 || ErrorTempAirOut || ErrorTempAirIn || ErrorTempTEH){
    AT_state=1; AT_Abort(-1);
    return;
  }
  AT_step = step;
  float base = constrain(TEHPower+TEHPower_PIDCorrection, 0, 10); //current power is a good middle point
  AT_high = min(base+AT_step, 10.0f);
  AT_low  = max(base-AT_step, 0.0f);
  AT_relayHigh = (tempAirOut < AirOutTargetTemp);
  AT_highSwitches=0; AT_cycles=0; AT_sumAmp=0; AT_sumPeriod=0;
  AT_max=-100; AT_min=1000;
  AT_start = AT_lastSwitch = millis();
  AT_state=1;
  AT_Report();
}
void AT_Finish(){
  float d = (AT_high-AT_low)/2;                    //relay amplitude (power)
  float a = AT_sumAmp/AT_cycles/2;                 //air-out oscillation amplitude (*C)
  float Tu = AT_sumPeriod/(float)AT_cycles/1000.0f; //oscillation period (sec)
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_AT_Amp, fround(a,2));
  if(a <= AT_hyst*1.2f){ AT_Abort(-4); return; }   //oscillation hidden in hysteresis/noise: use bigger step
  float Ku = 4*d/(PI*sqrt(a*a-AT_hyst*AT_hyst));   //ultimate gain (describing function, hysteresis corrected)
  //Tyreus-Luyben PI: less aggressive than Ziegler-Nichols, good for slow processes with lag
  TEHPID_Kp = Ku/3.2f;
  TEHPID_Ki = TEHPID_Kp/(2.2f*Tu);
  TEHPID_Kd = 0;
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_AT_Ku, fround(Ku,3));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_AT_Tu, fround(Tu,0));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_Kp, fround(TEHPID_Kp,3));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_Ki, TEHPID_Ki);
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_Kd, TEHPID_Kd);
  EEPROM_storeValues();
  //continue with PID bumplessly from the middle of relay:
  TEHPID_Reset();
  TEHPID_I = (AT_high+AT_low)/2 - TEHPower;
  TEHPower_PIDCorrection = TEHPID_I;
  AT_state=100;
  AT_Report();
}
void AT_Step(){ //on each new air-out reading
  unsigned long now=millis();
  if(now-AT_start > AT_MAX_TOTAL_MS){ AT_Abort(-3); return; }
  if(now-AT_lastSwitch > AT_MAX_HALFCYCLE_MS){ AT_Abort(-2); return; }
  if(tempAirOut>AT_max) AT_max=tempAirOut;
  if(tempAirOut<AT_min) AT_min=tempAirOut;

  if(AT_relayHigh && tempAirOut > AirOutTargetTemp+AT_hyst){
    AT_relayHigh=0;
    AT_lastSwitch=now;
  }else if(!AT_relayHigh && tempAirOut < AirOutTargetTemp-AT_hyst){
    AT_relayHigh=1;
    AT_lastSwitch=now;
    AT_highSwitches++;
    //full cycle = between switches to high; 1st one is transient - skip it
    if(AT_highSwitches>=3){
      AT_sumAmp += AT_max-AT_min;
      AT_sumPeriod += now-AT_lastHighSwitch;
      AT_cycles++;
      AT_Report();
      if(AT_cycles>=AT_CYCLES_NEEDED){
        AT_lastHighSwitch=now;
        AT_Finish();
        return;
      }
    }
    AT_lastHighSwitch=now;
    AT_max=-100; AT_min=1000;
  }
}

void TEHPIDCorrectionEvaluation(){ //calc TEHPower_PIDCorrection, so that TEHPower+correction is in 0..10
  if(kPwr_PreheatIsOn==1){
    TEHPower_PIDCorrection=0;
    return;
  }

  if(AT_state==1){ //autotune: relay output instead of PID
    if(newAirOutReading){
      newAirOutReading=0;
      AT_Step();
    }
    if(AT_state==1){
      TEHPower_PIDCorrection = (AT_relayHigh ? AT_high : AT_low) - TEHPower;
      addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower+TEHPower_PIDCorrection,1));
      return;
    }
  }

  if(!newAirOutReading) //PID step only on a new measurement (temps are read every ReadTempCycleInterval)
    return;
  newAirOutReading=0;

  unsigned long now=millis();
  if(isnan(TEHPID_prevtempAirOut)){ //first step after start
    TEHPID_prevtempAirOut = tempAirOut;
    TEHPID_prevMillis = now;
  }
  float dt = (now-TEHPID_prevMillis)/1000.0f; //sec
  if(dt>60) dt=60; //after sensor errors - don't make a huge integral step
  TEHPID_prevMillis = now;

  float err = AirOutTargetTemp - tempAirOut;
  float P = TEHPID_Kp * err;
  float D = (dt>0) ? -TEHPID_Kd * (tempAirOut-TEHPID_prevtempAirOut)/dt : 0; //on measurement: no kick when target changes
  TEHPID_prevtempAirOut = tempAirOut;

  //anti-windup: integrate only if it doesn't push power further into saturation
  float newI = TEHPID_I + TEHPID_Ki*err*dt;
  float out = TEHPower + P + newI + D;
  if(!((out>10 && err>0) || (out<0 && err<0)))
    TEHPID_I = constrain(newI, -10, 10);
  out = constrain(TEHPower + P + TEHPID_I + D, 0, 10);
  TEHPower_PIDCorrection = out - TEHPower;

  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_P, fround(P,2));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_I, fround(TEHPID_I,2));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPID_D, fround(D,2));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround(TEHPower+TEHPower_PIDCorrection,1));
  addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower_PIDCorrection, fround(TEHPower_PIDCorrection,2));
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

  if(tempAirOut > AirOutTargetTemp_MAX+2){ //additional protection
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
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); //MAX6675: max 4.3MHz
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
  SPI.endTransaction(); //CAN CS stays HIGH (unselected): mcp_can selects it itself
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
    newAirOutReading=1;
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
  if(isnan(tempTEH)){ //thermocouple disconnected (x==NAN is always false)
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
void ValveCloseDelayed(){ //timer callback: forget its id first, or ValveClose would delete this running timer's slot
  valveCloseTimerId = -1;   //and its new stop timer could get the same slot and be deleted by timer.run()
  ValveClose();
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
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_TEHPower, fround((TEHPIDSTATUS>0 ? TEHPower+TEHPower_PIDCorrection : 0), 1));
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
          valveCloseTimerId = timer.setTimeout(20*1000L, ValveCloseDelayed);
        }else{//read KTC and decide weather TEH is cooled enough:
          if(tempTEH>50){
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
        if(ErrorTempTEH==0 && tempTEH>50){
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
          if(TEHPIDSTATUS!=1){ //its turning on (from off or kPwr mode)
            TEHPID_Reset();
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

  if(TEHPIDSTATUS!=1)
    AT_Abort(-1); //autotune only in PID heating mode

  if((TEHPIDSTATUS==0 && PROTECTIONRELAYSTATUS==0) || errorTEHOverheatError!=0){
    digitalWrite(PROTECTION_ON_PIN, LOW);
    TEHPID_I=0; //clear integral part of PID
  }else{
	  if(errorTEHOverheatError==0){
      digitalWrite(PROTECTION_ON_PIN, HIGH);
	  }
  }//till next run time - relay will have time to switch
}

//////////////////////CAN commands///////////////////
char ProcessReceivedVirtualPinString(unsigned char vPinNumber, char* tmp, unsigned char len){ return 0; } //empty
char ProcessReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
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
    case VPIN_SetTEHPID_Isum_Zero: TEHPID_I=0; break;
    case VPIN_TEHPID_AutoTune:
      if(vPinValueFloat<0.5f)
        AT_Abort(0); //0 = stopped by command
      else if(AT_state!=1)
        AT_Start(vPinValueFloat<1.5f ? 3 : constrain(vPinValueFloat,1.5f,5)); //power step (of 0..10)
      break;
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
void EEPROM_storeValues(){ //EEPROM.put writes only changed bytes
  InsureSafeValues();
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
  EEPROM.put(VPIN_TEH_KdTempAirIn*sizeof(float),   KdT_TEH);
  EEPROM.put(VPIN_HEATER_TRGSTATUS*sizeof(float),  targetHeaterStatus);
  
  //EEPROM.put(VPIN_PIDSTATUS*sizeof(float),      TEHPIDSTATUS);
  //EEPROM.put(VPIN_VALVESTATUS*sizeof(float),    VALVESTATUS);
  
}
float EEPROM_validFloat(float v, float vmin, float vmax, float vdefault){
  return (isnan(v) || v<vmin || v>vmax) ? vdefault : v;
}
void EEPROM_restoreValues(){
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
  EEPROM.get(VPIN_TEH_KdTempAirIn*sizeof(float),   KdT_TEH);
  EEPROM.get(VPIN_HEATER_TRGSTATUS*sizeof(float),  targetHeaterStatus);
  
  //EEPROM.get(VPIN_PIDSTATUS*sizeof(float),         TEHPIDSTATUS);
  //EEPROM.get(VPIN_VALVESTATUS*sizeof(float),       VALVESTATUS);
  if(TEHPowerPeriodSeconds<1 || TEHPowerPeriodSeconds>60) TEHPowerPeriodSeconds=2;
  TEHPeriodMillis = (unsigned long)TEHPowerPeriodSeconds*1000L;
  TEHPID_Kp = EEPROM_validFloat(TEHPID_Kp, 0, 100, 1);
  TEHPID_Ki = EEPROM_validFloat(TEHPID_Ki, 0, 10, 0.003);
  TEHPID_Kd = EEPROM_validFloat(TEHPID_Kd, 0, 1000, 0);
  TEHPower  = EEPROM_validFloat(TEHPower, 0, 10, 0);
  AirOutTargetTemp   = EEPROM_validFloat(AirOutTargetTemp, AirOutTargetTemp_MIN, AirOutTargetTemp_MAX, 20);
  kPwr2Air           = EEPROM_validFloat(kPwr2Air, 0, 10, 0.34);
  kPwr_preMillisPerC = EEPROM_validFloat(kPwr_preMillisPerC, 0, 60000, 4000);
  KdT_TEH            = EEPROM_validFloat(KdT_TEH, minKdT_TEH, maxKdT_TEH, 2);
  if(targetHeaterStatus<0 || targetHeaterStatus>5) targetHeaterStatus=0;
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
  //if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
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
  CAN0.init_Filt(1,0,0x01000000);

  CAN0.init_Mask(1,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(2,0,0x01000000);
  CAN0.init_Filt(3,0,0x01000000);
  CAN0.init_Filt(4,0,0x01000000);
  CAN0.init_Filt(5,0,0x01000000);
  // #ifdef testmode
  // CAN0.init_Filt(1,0,filt1);                // Init second filter...
  // #endif
  
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
