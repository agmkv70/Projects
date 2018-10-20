#include <Arduino.h>

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile int ari;
volatile int ar[512];
volatile int channel;
volatile unsigned long startMicros,endMicros;

void setup()
{  //TIMSK0 = 0x00; // отключаем таймер (из-за прерываний)
   //DIDR0 = 0x3F; // отключаем цифровые входы

   //опорное AVCC (5V) для ATmega328
   //установим 01
   ADMUX =0; //&= ~((1<<REFS1) | (1<<REFS0));//очистили оба бита
   ADMUX |= (1<<REFS0); //01

   //ADCSRA = 0xAC; // 1010 1100 включаем АЦП ADEN, разрешаем прерывания ADFR, делитель = 128 111
   ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)
            |(1<<ADIE)
            |(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
    // set prescale to  64 110 //faster, medium (lower is more than 250kHz - so is not precise)
    // set prescale to 128 111 //slow, fine
    // sbi(ADCSRA,ADPS2) ;
    // sbi(ADCSRA,ADPS1) ;
    // cbi(ADCSRA,ADPS0) ;
   //ADCSRB = 0x40; // Включаем каналы MUX АЦП, режим постоянной выборки

   Serial.begin(115200);
   Serial.println("Start");

  channel=0;
  sbi(ADCSRA, ADSC); // Запускаем преобразование установкой бита 6 (=ADSC) в ADCSRA
  sei(); // устанавливаем глобальный флаг прерываний

  //for(int i=0;i<500;i++)
  //   Serial.println(ar[i]);
}

unsigned long lasttimerun;
volatile int resIsPrinted;
void loop()
{
  // for(int i=0;i<500;i++)
  //  {  ar[i]=analogRead(0);
  //    delayMicroseconds(70);
  //  }
  if(resIsPrinted==0)
  { resIsPrinted=1;
    for(int i=0;i<8;i++) //8 of 512
    { Serial.println(ar[i]);
    }
  }
  unsigned long curMillis=millis();
  if(lasttimerun+4000 < curMillis)
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

ISR(ADC_vect)
{ if(ari>=512) return; //just skip
  if(ari==0) startMicros=micros();
  channel = (++channel)%8;
  ADMUX=(1<<REFS0)|(channel);
  delayMicroseconds(10);
  sbi(ADCSRA, ADSC); //start next convertion

  int analogValue = ADCL; // сохраняем младший байт АЦП
  analogValue += ADCH << 8; // сохраняем старший байт АЦП
  ar[ari] = analogValue;

  ari++;
  //Serial.println(ar[i]);

  if(ari>=512)
  { resIsPrinted=0; //уже можно печатать
    endMicros=micros();
  }
}
