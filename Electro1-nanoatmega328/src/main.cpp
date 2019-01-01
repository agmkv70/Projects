#include <Arduino.h>

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define NChannels 1
volatile int ari;
volatile int ar[512*NChannels];
volatile int channel;
volatile unsigned long startMicros,endMicros;

void setup()
{ Serial.begin(115200);
  Serial.println("Start");
  
  pinMode(PIN_A0, INPUT);
  pinMode(PIN_A1, INPUT);
  pinMode(PIN_A2, INPUT);
  pinMode(PIN_A3, INPUT);
  pinMode(PIN_A4, INPUT);
  pinMode(PIN_A5, INPUT);
  pinMode(PIN_A6, INPUT);
  pinMode(PIN_A7, INPUT);

  return;

  /////////////Настройка АЦП://///////////
  //TIMSK0 = 0x00; // отключаем таймер (из-за прерываний)
  //DIDR0 = 0x3F; // отключаем цифровые входы

  //опорное AVCC (5V) для ATmega328
  //установим 01
  //ADMUX =0; //&= ~((1<<REFS1) | (1<<REFS0));//очистили оба бита
  //ADMUX |= ((0<<REFS1) | (1<<REFS0)); //01 - тут просто берется напряжение питания
  channel=0; 
  ADMUX=(0<<REFS1) | (1<<REFS0) | (channel); //01=

  //ADCSRA = 0xAC; // 1010 1100 включаем АЦП ADEN, разрешаем прерывания ADFR, делитель = 128 111
  ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF) //ADATE =  Auto Triggering; The ADC will start a conversion on a positive edge of the selected trigger signal. The trigger source is selected by setting the ADC Trigger Select bits, ADTS in SFIOR.
          |(1<<ADIE)
          |(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
  // set prescale to  64 110 //faster, medium (lower is more than 250kHz - so is not precise)
  // set prescale to 128 111 //slow, fine
  // sbi(ADCSRA,ADPS2) ;
  // sbi(ADCSRA,ADPS1) ;
  // cbi(ADCSRA,ADPS0) ;
  //ADCSRB = 0x40; // Включаем каналы MUX АЦП, режим постоянной выборки
  
  sbi(ADCSRA, ADSC); // Запускаем преобразование установкой бита 6 (=ADSC) в ADCSRA
  sei(); // устанавливаем глобальный флаг прерываний
}

unsigned long lasttimerun;
volatile int resIsPrinted;

void loop(){ 
  for(int i=0;i<512;i++)
  { ar[i]=analogRead(0);
    delayMicroseconds(70);
  }
  resIsPrinted=0;
  Serial.print("//////////////////////////////////////////////");

  if(resIsPrinted==0)
  { resIsPrinted=1;
    for(int i=0;i<512;i++) //8 of 512
    { Serial.println(ar[i]);
    }
  }
  delay(10000);
  return;

  unsigned long curMillis=millis();
  if(lasttimerun+10000 < curMillis)
  { ari=0; //начало
    lasttimerun=curMillis;
    Serial.print("run ");
    Serial.println(lasttimerun);
    Serial.println(resIsPrinted);
    Serial.print("end-start micros = ");
    Serial.println((endMicros-startMicros));
    sbi(ADCSRA, ADSC); //start next convertion session
  }

}

ISR(ADC_vect){
   if(ari>=512*NChannels) return; //just skip
  if(ari==0) startMicros=micros();

  channel = (++channel)%8;
  ADMUX=(0<<REFS1) | (1<<REFS0) | (channel);

  delayMicroseconds(10); 
  sbi(ADCSRA, ADSC); //start next convertion

  int analogValue = (ADCH << 8) | ADCL; // read both bytes (in ADCH only 2 lower bits are used)
  ar[ari] = analogValue;

  ari++;
  //Serial.println(ar[i]);

  if(ari>=512*NChannels)
  { resIsPrinted=0; //уже можно печатать
    endMicros=micros();
  }
}
