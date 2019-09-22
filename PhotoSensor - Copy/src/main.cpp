#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

const int pinPhoto = A0;
int val;

//#define FASTADC 0
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define arSize 400
int ar[arSize];

#define zerolux 100

void setup() {
    Serial.begin(115200);
    pinMode( pinPhoto, INPUT );

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64) 

    #if FASTADC
    // set prescale to 16 scc=76.8kHz; sss=9.6kHz
    sbi(ADCSRA,ADPS2) ;
    sbi(ADCSRA,ADPS1) ;
    sbi(ADCSRA,ADPS0) ;
    #endif
}

void loop() {
    long t0 = millis();
    long sum=0;
    int i;
    for(i=0;i<arSize;i++)
    {   val = analogRead( pinPhoto );
        ar[i] = val;
        sum += val;
        delayMicroseconds(200);
    }
    long readtime = millis()-t0;
    int med = sum/i;

    int state=0, numZeroUp=0, difsum=0;
    for(i=0;i<arSize;i++)
    {   int dif = ar[i]-med;
        if(dif>0)
            if(state<=0)
                numZeroUp += 1; //go through 0 upwards
        if(dif<0)
            difsum += -dif;
        else
            difsum += dif;
        state = dif;
    }

    float amp = 0;
    float period=readtime;
    float Hz=0;
    if(numZeroUp!=0 && readtime!=0)
    {   period = (float)readtime/(float)numZeroUp; //millis
        Hz = 1000*(float)numZeroUp/(float)readtime; //millis
        amp = (float)difsum/(float)arSize;
    }

    Serial.print( "Overall read time=" );
    Serial.print( readtime );
    Serial.print( " per=" );
    Serial.print( period );
    Serial.print( " ampAvg=" );
    Serial.print( amp );
    Serial.print( " Hz=" );
    Serial.print( round(Hz) );
    Serial.print( " med=" );
    Serial.println( med );
    
    display.clearDisplay(); // Clear the buffer
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0); // Устанавливаем курсор в колонку 0, строку 0. на самом деле это строка №1, т.к. нумерация начинается с 0.
    display.print("A0 amp:");
    if(amp<2) 
    {   display.print(" LOW AMPLITUDE");
        Hz=0;
    }else{
        display.print(round(amp), DEC);
        display.print(" amp");
    }
    
    display.setTextSize(2);
    display.setCursor(0,17); 
            
    display.print(round(Hz), DEC);
    display.print(" Hz ");

    display.setTextSize(1);
    //display.setCursor(0,17); 
    display.print(med, DEC);
    display.print(" lvl");
    display.display();

    delay(200);
}