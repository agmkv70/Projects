#include "Arduino.h"

class MAX6675 {
 public:
  MAX6675(int8_t SCLK, int8_t CS, int8_t MISO);

  double readCelsius(void);
  private:
  int8_t sclk, miso, cs;
  uint8_t spiread(void);
};


#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

MAX6675::MAX6675(int8_t SCLK, int8_t CS, int8_t MISO) {
  sclk = SCLK;
  cs = CS;
  miso = MISO;

  //define pin modes
  pinMode(cs, OUTPUT);
  pinMode(sclk, OUTPUT); 
  pinMode(miso, INPUT);

  digitalWrite(cs, HIGH);
}
double MAX6675::readCelsius(void) {

  uint16_t v;

  digitalWrite(cs, LOW);
  _delay_ms(1);

  v = spiread();
  v <<= 8;
  v |= spiread();

  digitalWrite(cs, HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN; 
    //return -100;
  }

  v >>= 3;

  return v*0.25;
}

byte MAX6675::spiread(void) { 
  int i;
  byte d = 0;

  for (i=7; i>=0; i--)
  {
    digitalWrite(sclk, LOW);
    _delay_ms(1);
    if (digitalRead(miso)) {
      //set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(sclk, HIGH);
    _delay_ms(1);
  }

  return d;
}


int thermoDO = 12;
int thermoCS = PIN_A4;
int thermoCLK = 13;
#define CAN_PIN_CS 10
  

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup1(){
  Serial.begin(115200);
  
  pinMode(CAN_PIN_CS,OUTPUT);
  digitalWrite(CAN_PIN_CS, HIGH);
  
  //pinMode(LED_PIN,OUTPUT);
  //digitalWrite(LED_PIN,LOW); //turn off LED

  Serial.println("MAX6675 test:");
  // wait for MAX chip to stabilize
  delay(100);
}

void loop1(){
   Serial.print("C = "); 
   Serial.println(thermocouple.readCelsius());
   
   delay(2000);
}