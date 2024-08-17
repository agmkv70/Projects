#include <Arduino.h>
#include <SimpleTimer.h>
//#define _serialprint //turns on printing messages

#include "ode2joy.h"
int melodyPlay=0, currNote=0;
int *melody,num_notes;
long melodyPauseBeforeRepeat=0;

SimpleTimer mel_timer;
SimpleTimer timer2;

#define PIN_Buzzer 7
#define PIN_LEDOnShield1 11 //yellow ?
#define PIN_LEDOnShield2 12 //red ?
#define PIN_BUTTON 2
#define PIN_RELAY1off 8
#define PIN_RELAY2on 9
#define PIN_MEASUREVOLT A0

int LEDState=0;
long timeLED;
long LEDFlashOn=10,LEDFlashOff=5000;
void LEDFlashMode(byte mode){
  if(mode==0){ //wait
    LEDFlashOn=10;
    LEDFlashOff=5000;
  }else if(mode==1){ //high
    LEDFlashOn=100;
    LEDFlashOff=40;
  }else if(mode==2){ //low
    LEDFlashOn=200;
    LEDFlashOff=1000;
  }
}
//left yellow(green?) LED
int LEDState2=0;
long timeLED2;
long LEDFlashOn2=10,LEDFlashOff2=5000;
void LEDFlashMode2(byte mode){
  if(mode==0){ //wait (off)
    LEDFlashOn2=10;
    LEDFlashOff2=5000;
  }else if(mode==1){ //high (charging)
    LEDFlashOn2=500;
    LEDFlashOff2=500;
  }else if(mode==2){ //low  (charged (off))
    LEDFlashOn2=10;
    LEDFlashOff2=200;
  }
}
long buttonDownDebounce=0;
int buttonPressed=0;

float voltage=0, voltageCoef=0.015157616f;
const int VArNumReadings = 10;
float VArReadings[VArNumReadings]; //array for sliding average
int VArIndex = 0;                   //current index
float VArSumTotal = 0;
long startTime;                

#define BatteryConnectedV 7
#define DischargeAlarmV 12.4 //12.4
#define DischargeAlarmReset 12.6 //12.7

#define EnableAutoChargeV 13.5 //auto can be turned on
#define OffAutoChargeV 13.8 //turn off charging

#define OverchargeAlarmV 14.2  //14.4
#define OverchargeAlarmReset 14.0  //14.2

/*#define VChangeToBalance 0.1 //as small as i can reliably detect
#define BalanceTime 7200000 //2hours (2*3600sec*1000millis)
float lastBalancedV=0;
long startBalanceTime=0;
*/

enum StatesMelodyPlay {st_off,st_melodyPlay,st_forceStopped}; //forceStopped=Silenced (LED continues flashing)
StatesMelodyPlay state_discharged=st_off; 
StatesMelodyPlay state_charged=st_off; 
int state_AutoCharging=0; // 0/1/-1(fast blinking charged) - starts when charge button pressed, ends via button or high voltage - priority state (when =1)

void repeatMelody();
void timer_call_playmelody() { //////////////////
  /*if(melodyPlay){
    if(currNote==3){//COUNT_NOTES){
      noTone(PIN_Buzzer);
      digitalWrite(PIN_Buzzer,HIGH);
      melodyPlay=0;
    }else{
      //noTone(PIN_Buzzer);
      digitalWrite(PIN_Buzzer,HIGH);
      //delay(20);
      tone(PIN_Buzzer, tones[currNote], durations[currNote]);
      mel_timer.setTimeout(durations[currNote],timer_call_playmelody);
      currNote++;
    }
  }*/
  if(melodyPlay){
    if(currNote>=num_notes){//COUNT_NOTES){
      noTone(PIN_Buzzer);
      digitalWrite(PIN_Buzzer,HIGH);
      
      if(melodyPauseBeforeRepeat>0){ //its almost playing, so we do not turn off melodyPlay :)
        mel_timer.setTimeout(melodyPauseBeforeRepeat,repeatMelody);
      }else{
        melodyPlay=0;
      }
    }else{
      //noTone(PIN_Buzzer);
      //digitalWrite(PIN_Buzzer,HIGH);
      //delay(20);
      
      int divider = melody[currNote*2 + 1], noteDuration=0;
      if (divider > 0) {
        noteDuration = (wholenote) / divider;
      } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration = (wholenote) / (-divider);
        noteDuration *= 1.5; // increases the duration in half for dotted notes
      }

      // we only play the note for 90% of the duration, leaving 10% as a pause
      tone(PIN_Buzzer, melody[currNote*2], noteDuration*0.9);

      mel_timer.setTimeout(noteDuration,timer_call_playmelody);
      currNote++; 
    }
  }/*else{
    noTone(PIN_Buzzer);
    digitalWrite(PIN_Buzzer,HIGH);
  }*/
}

void startMelody(byte mel_type,long aMelodyPauseBeforeRepeat=0){
  ///////////start melody//////////
  melodyPlay=1;
  currNote=0;
  melodyPauseBeforeRepeat = aMelodyPauseBeforeRepeat;

  if(mel_type==1){ //oda to joy
    melody = melody_oda2joy;
    num_notes=sizeof(melody_oda2joy)/sizeof(melody_oda2joy[0])/2; 
    LEDFlashMode(1);
    LEDFlashMode2(0);

  }else if(mel_type==2){ //bach toccata d minor
    melody = melody_bachtocc;
    num_notes=sizeof(melody_bachtocc)/sizeof(melody_bachtocc[0])/2; 
    LEDFlashMode(2);
    LEDFlashMode2(0);
  }else if(mel_type==3){ //LionSleepsTonight
    melody = melody_LionSleepsTonight;
    num_notes=sizeof(melody_LionSleepsTonight)/sizeof(melody_LionSleepsTonight[0])/2; 
    LEDFlashMode2(2);
  }

  noTone(PIN_Buzzer);
  digitalWrite(PIN_Buzzer,HIGH);
  for(int i=0;i<mel_timer.getNumTimers();i++){
    mel_timer.deleteTimer(i);
  }
  timer_call_playmelody();
}

void repeatMelody(){
  currNote=0;
  timer_call_playmelody();
}

void stopMelody(){
  melodyPlay=0;
  noTone(PIN_Buzzer);
  digitalWrite(PIN_Buzzer,HIGH);
  melodyPauseBeforeRepeat=0;
}

void onButtonPressed() { ////////////////////////
  
  if(state_AutoCharging!=0){ //turn off charging or fast blinking (charged)
    state_AutoCharging=0;
     //off autocharging:
    digitalWrite(PIN_RELAY1off,HIGH);
    delay(4);
    digitalWrite(PIN_RELAY1off,LOW);
    LEDFlashMode2(0);
    return;
  }

  if(state_discharged==st_melodyPlay){
    state_discharged = st_forceStopped;
    stopMelody();

    tone(PIN_Buzzer, 4000, 20);
    delay(20);
    noTone(PIN_Buzzer);
    //delay(20);
    tone(PIN_Buzzer, 2000, 20);
    delay(20);
    noTone(PIN_Buzzer);
    digitalWrite(PIN_Buzzer,HIGH);
  }else if(state_charged==st_melodyPlay){
    state_charged = st_forceStopped;
    stopMelody();

    tone(PIN_Buzzer, 4000, 20);
    delay(20);
    noTone(PIN_Buzzer);
    //delay(20);
    tone(PIN_Buzzer, 2000, 20);
    delay(20);
    noTone(PIN_Buzzer);
    digitalWrite(PIN_Buzzer,HIGH);
  }else if(state_AutoCharging<=0 && voltage < EnableAutoChargeV){ //turn on charging
    state_AutoCharging=1;
    //on autocharging:
    digitalWrite(PIN_RELAY2on,HIGH);
    delay(4);
    digitalWrite(PIN_RELAY2on,LOW);
    LEDFlashMode2(1);
    return;
  }

}

void OnVoltageMeasured(){ ///////////////////////

  if(state_AutoCharging==1 && voltage > OffAutoChargeV){ //CHARGED! -> turn off auto charging
    state_AutoCharging=-1;
     //off autocharging:
    digitalWrite(PIN_RELAY1off,HIGH);
    delay(4);
    digitalWrite(PIN_RELAY1off,LOW);
    LEDFlashMode2(0);

    startMelody(3,0); //lion sleeps tonight, no repeat (0 interval)
  }

  //discharged:
  if(BatteryConnectedV < voltage && voltage <= DischargeAlarmV){ //low voltage -> discharged
    if(state_discharged==st_off){
      state_discharged=st_melodyPlay;
    }
  }
  if(voltage >= DischargeAlarmReset && state_discharged != st_off){ //low voltage melody turn off
    state_discharged=st_off;
    stopMelody();
    LEDFlashMode(0);
  }
  if(state_discharged == st_melodyPlay && melodyPlay==0){
    startMelody(2,20000); //dMinor
    LEDFlashMode(2);
  }

  //charged:
  if(voltage > OverchargeAlarmV){ //high voltage -> charged
    if(state_charged==st_off){
      state_charged=st_melodyPlay;
    }
  }
  if(voltage <= OverchargeAlarmReset && state_charged != st_off){ //high v melody turn off
    state_charged=st_off;
    stopMelody();
    LEDFlashMode(0);
  }
  if(state_charged == st_melodyPlay && melodyPlay==0){
    startMelody(1,10000); //joy
    LEDFlashMode(1);
  }

  /*
  //balancing:
  if(abs(voltage-lastBalancedV) > VChangeToBalance){
    lastBalancedV = voltage; 
    startBalanceTime=millis(); 
    //on balancer
    digitalWrite(PIN_RELAY2on,HIGH);
    delay(4);
    digitalWrite(PIN_RELAY2on,LOW);
  }
  if(startBalanceTime!=0 && millis()-startBalanceTime > BalanceTime){
    startBalanceTime = 0;
    //off balancer
    digitalWrite(PIN_RELAY1off,HIGH);
    delay(4);
    digitalWrite(PIN_RELAY1off,LOW);
  }*/
}

void measureVoltage(){ //////////////////////////
  int val = analogRead(PIN_MEASUREVOLT);
  float curV = (float)val*voltageCoef;

  //sliding average:
  VArSumTotal = VArSumTotal - VArReadings[VArIndex]; 
  VArReadings[VArIndex] = curV; 
  VArSumTotal = VArSumTotal + VArReadings[VArIndex];       
  VArIndex++;
  if (VArIndex >= VArNumReadings){             
    VArIndex = 0;                           
  }
  voltage = VArSumTotal / VArNumReadings; 
  
  #ifdef _serialprint
  Serial.print(val);
  Serial.print(" - ");
  Serial.print(curV,2);
  Serial.print(" - ");
  Serial.print(voltage,2);
  Serial.println();
  #endif

  if(millis()-startTime>3500) //skip actions till voltage sliding window is fully populated
    OnVoltageMeasured();
}

void setup() { ////////////////////////////////////////////////
  #ifdef _serialprint
  Serial.begin(9600);
  #endif
  startTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  pinMode(PIN_Buzzer, OUTPUT);
  digitalWrite(PIN_Buzzer,HIGH);//HIGH=transistor off (inverted)
  
  timeLED = millis();
  pinMode(PIN_LEDOnShield1, OUTPUT);
  pinMode(PIN_LEDOnShield2, OUTPUT);
  digitalWrite(PIN_LEDOnShield1,LOW);
  digitalWrite(PIN_LEDOnShield2,LOW);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  //pinMode(PIN_LEDButton, INPUT);
  
  for(int i=0;i<VArNumReadings;i++){
    VArReadings[i]=0;
  }
  analogReference(INTERNAL);
  timer2.setInterval(300,measureVoltage);

  pinMode(PIN_RELAY1off, OUTPUT);
  digitalWrite(PIN_RELAY1off,LOW);
  pinMode(PIN_RELAY2on, OUTPUT);
  digitalWrite(PIN_RELAY2on,LOW);
  
  //off autocharging
  digitalWrite(PIN_RELAY1off,HIGH);
  delay(4);
  digitalWrite(PIN_RELAY1off,LOW);

  LEDFlashMode(0);
  LEDFlashMode2(0);
}

void loop() { /////////////////////////////////////////////////

  mel_timer.run();
  timer2.run();

  long LEDDuration=millis()-timeLED;
  if(LEDState>0){
    if(LEDDuration>LEDFlashOn){
      LEDState=0;
      digitalWrite(PIN_LEDOnShield1,LOW);
      timeLED = millis();
    }
  }else{
    if(LEDDuration>LEDFlashOff){
      LEDState=1;
      digitalWrite(PIN_LEDOnShield1,HIGH);
      timeLED = millis();
    }
  }
  //second (left,green/yellow) LED
  long LEDDuration2=millis()-timeLED2;
  if(LEDState2>0){
    if(LEDDuration2>LEDFlashOn2){
      LEDState2=0;
      digitalWrite(PIN_LEDOnShield2,LOW);
      timeLED2 = millis();
    }
  }else{
    if(LEDDuration2>LEDFlashOff2){
      LEDState2=1;
      digitalWrite(PIN_LEDOnShield2,HIGH);
      timeLED2 = millis();
    }
  }

  //button:
  if(digitalRead(PIN_BUTTON)==LOW){ //pressed ?
    if(buttonPressed==1){
      ;
    }else if(buttonDownDebounce==0){ //first time
      buttonDownDebounce=millis(); //start debouncing time
    }else if (millis() - buttonDownDebounce >= 50){
      onButtonPressed();
      buttonPressed=1;
    }
  }else{ //button up
    buttonPressed=0;
    buttonDownDebounce=0;
  }
  
} //loop

