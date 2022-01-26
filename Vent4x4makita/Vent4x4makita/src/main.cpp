#include <Arduino.h>

void setup() {
  /*
  //для подключения к счетчику через ардуинку - ТХ=0, РХ=1
  pinMode(0, INPUT); 
  pinMode(1, INPUT);*/ 
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(A0, INPUT); 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);
  
  //Serial.begin(9600);
}

long loopTime = 1000000/(21*1000), timeread=0;
int val = 10;

void loop() {
  //delay(100);

  unsigned long timenow = millis();
  if(timenow-timeread>100)
  { val = analogRead(A0);
    //Serial.print(val); Serial.print(" = "); 
    timeread = timenow;
    //if(val<255)
    //  val=255; //20% ?
    val = (long)(255+val) * loopTime / (1023+255);
    //Serial.print(round(val*100/loopTime)); Serial.print(" : "); Serial.print(val); Serial.print(" - "); Serial.println(loopTime-val);
  }
    
  digitalWrite(8,HIGH);
  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(12,HIGH);
  delayMicroseconds(val);
  digitalWrite(8,LOW);
  digitalWrite(LED_BUILTIN,LOW);
  digitalWrite(12,LOW);
  delayMicroseconds(loopTime-val);

}