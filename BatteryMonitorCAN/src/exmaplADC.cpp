/*#include <avr/io.h>
#include <avr/interrupt.h>

const int mainsMonitorRefPin = 0;
static volatile unsigned char currentAdcChannel;

void setup()
{
  // Set up the ADC. We need to read the mains voltage frequently in order to get a sufficiently accurate RMS reading.
  // To avoid using too much CPU time, we use the conversion-complete interrupt. This means we can't use analogRead(),
  // we have to access the ADC ports directly. 
  currentAdcChannel = mainsMonitorRefPin;      // set up which analog input we will read first
  ADMUX = 0b01000000 | currentAdcChannel;
  ADCSRB = 0b00000000; // Analog Input bank 1
  ADCSRA = 0b10011111; // ADC enable, manual trigger mode, ADC interrupt enable, prescaler = 128
  
  // Set up a timer 2 interrupt every 1ms to kick off ADC conversions etc.
  // Do this last after we have initialized everything else
  ASSR = 0;
  TCCR2A = (1 << WGM21);    // CTC mode
  TCCR2B = (1 << CS22);     // prescaler = 64
  TCNT2 = 0;                // restart counter
  OCR2A = 249;              // compare register = 249, will be reached after 1ms
  TIMSK2 = (1 << OCIE2A);
}

// Interrupt service routine for the 1ms tick
ISR(TIMER2_COMPA_vect)
{
  TIFR2 = (1 << OCF2B);  // shouldn't be needed according to the documentation, but is (perhaps the ISR is too short without it?)
  // Kick off a new ADC conversion. We already set the multiplexer to the correct channel when the last conversion finished.
  ADCSRA = 0b11001111;   // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128
}

// Interrupt service routine for ADC conversion complete
ISR(ADC_vect) 
{
  // The mcu requires us to read the ADC data in the order (low byte, high byte)
  unsigned char adcl = ADCL;
  unsigned char adch = ADCH;
  unsigned int adcVal = (adch << 8) | adcl;
  
  switch(currentAdcChannel)
  {
    case mainsMonitorRefPin:
      // do whatever you want with the reading in adcVal
      // set currentAdcChannel to the next pin you want to read
      //...
      break;

   // add switch cases fort the other channels you want to read here...
  }
  
  // Set the ADC multiplexer to the channel we want to read next. Setting if here rather than in the tick ISR allows the input
  // to settle, which is required for ADC input sources with >10k resistance.
  ADMUX = (0b01000000 | currentAdcChannel);   // Vcc reference, select current channel
}*/