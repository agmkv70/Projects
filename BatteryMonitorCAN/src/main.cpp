//#define testmode

#define CAN_PIN_INT 9
#define CAN_PIN_CS  10 

#include <NIK_defs.h>
#include <NIK_can.h>

#include <avr/io.h>
#include <avr/interrupt.h>

const int MonitorVPin = 2;
const int MonitorAPin = 3;
static volatile unsigned char currentAdcChannel;

float voltageV=0, voltageCoef=0.01550753; // divider-4.73(100k::(20k+6.8k)) maxV-15.6
float AverageV=0; //calculated as sliding average (of 10)
float currentA=0, currentCoef=-(0.5f); //-4=0; -45=+20.45A
const int curOffsetMidpoint=512-4;

float deltaTms=2; //1ms x2 = 2ms/500Hz
float SumDeltaWs=0;
float BAT_CurPowerW=0;
float BAT_EnergyWH=0,BAT_MaxEnergyWH=0,BAT_EnergyPercent=0;

const int VArNumReadings = 10;
float VArReadings[VArNumReadings]; //array for sliding average
int VArIndex = 0;                   //current index
float VArSumTotal = 0;
long startTime,lastSendTime=0,firstSend=1;                

int mainTimerId;
int eepromVIAddr=1000,eepromValueIs=7650+3; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=5; //seconds

/*void OnVoltageMeasured();

void measureVoltage(){ //////////////////////////
  int val = analogRead(PIN_MEASURE_V);
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
  
  #ifdef testmode
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

void OnVoltageMeasured(){ ///////////////////////
  //float voltage;

  #ifdef testmode
		Serial.print("voltage = ");
		Serial.println(voltageV);
  #endif

  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,fround(voltage,2)); //rounded 0.0 value
}
*/

void MainCycle_StartEvent(){
  #ifdef testmode
		Serial.print("----------V = ");
		Serial.print(fround(voltageV,2));
    Serial.print("   -------A = ");
		Serial.println(fround(currentA,2));
  #endif

  long curTime = millis();
  if(firstSend){
    firstSend = 0;
    lastSendTime = curTime;
    BAT_EnergyWH += SumDeltaWs/3600;
    SumDeltaWs = 0;
  }else{
    BAT_CurPowerW = SumDeltaWs / (curTime-lastSendTime)*1000; //average
    lastSendTime = curTime;
    BAT_EnergyWH += SumDeltaWs/3600;
    SumDeltaWs = 0;
    BAT_EnergyPercent = BAT_MaxEnergyWH==0 ? 0 : (BAT_EnergyWH / BAT_MaxEnergyWH * 100.0f);

    //calculate average V:
    VArSumTotal = 0;
    for(byte ii=0; ii<VArNumReadings; ii++){
      VArSumTotal += VArReadings[ii];
    }
    AverageV = VArSumTotal / VArNumReadings;

    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(voltageV,2));
    addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(AverageV,2)); 
    addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterA,fround(currentA,2));
    addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_CurPowerW,fround(BAT_CurPowerW,0));
    addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_EnergyPercent,fround(BAT_EnergyPercent,1));
  }
}

char ProcessReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
  //digitalWrite(13,HIGH);
  //delay(50);
  //digitalWrite(13,LOW);

	switch(vPinNumber){
		case VPIN_STATUS:
			boardSTATUS = (int)vPinValueFloat;
			EEPROM_storeValues();
			break;
		case VPIN_BAT_SendIntervalSec:
			if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
				break;
			MainCycleInterval = (int)vPinValueFloat;
			timer.deleteTimer(mainTimerId);
			mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
			EEPROM_storeValues();
			break;
		case VPIN_BAT_EnergyWH:  
      BAT_EnergyWH  = vPinValueFloat;  
      BAT_EnergyPercent = BAT_MaxEnergyWH==0 ? 0 : (BAT_EnergyWH / BAT_MaxEnergyWH * 100.0f);
      EEPROM_storeValues(); 
      break;
    case VPIN_BAT_MaxEnergyWH:  
      BAT_MaxEnergyWH  = vPinValueFloat;  
      BAT_EnergyPercent = BAT_MaxEnergyWH==0 ? 0 : (BAT_EnergyWH / BAT_MaxEnergyWH * 100.0f);
      EEPROM_storeValues(); 
      break;
    case VPIN_BAT_EnergyPercent:  
      BAT_EnergyPercent  = vPinValueFloat;
      BAT_EnergyWH = BAT_MaxEnergyWH * BAT_EnergyPercent / 100.0f;
      EEPROM_storeValues(); 
      break;
		default:
			return 0;
	}
	return 1;
}
char ProcessReceivedVirtualPinString(unsigned char vPinNumber, char* vPinString, byte dataLen){
  return 0;
}

void EEPROM_storeValues(){
  EEPROM_WriteAnything(eepromVIAddr,eepromValueIs);
  
  EEPROM.update(0,(unsigned char)boardSTATUS);
  EEPROM.update(10,(unsigned char)(MainCycleInterval));

  EEPROM_WriteAnything(VPIN_BAT_EnergyWH                ,(float)BAT_EnergyWH);
  EEPROM_WriteAnything(VPIN_BAT_EnergyWH+sizeof(float)*1,(float)BAT_MaxEnergyWH);
  EEPROM_WriteAnything(VPIN_BAT_EnergyWH+sizeof(float)*2,(float)BAT_EnergyPercent);
}
void EEPROM_restoreValues(){
  int ival;
  EEPROM_ReadAnything(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }

  boardSTATUS = EEPROM.read(0);
  
  int aNewInterval = EEPROM.read(10);
  if(aNewInterval != 0){
    MainCycleInterval = aNewInterval;
  }
  
  EEPROM_ReadAnything(VPIN_BAT_EnergyWH                ,BAT_EnergyWH);
  EEPROM_ReadAnything(VPIN_BAT_EnergyWH+sizeof(float)*1,BAT_MaxEnergyWH);
  EEPROM_ReadAnything(VPIN_BAT_EnergyWH+sizeof(float)*2,BAT_EnergyPercent);
}

void setupADC_and_TimerInt(){
   // Set up the ADC. We need to read the mains voltage frequently in order to get a sufficiently accurate RMS reading.
  // To avoid using too much CPU time, we use the conversion-complete interrupt. This means we can't use analogRead(),
  // we have to access the ADC ports directly. 
  currentAdcChannel = MonitorVPin;      // set up which analog input we will read first
  ADMUX = 0b00000000 | currentAdcChannel; //ext ref
  ADCSRB = 0b00000000; // Analog Input bank 1
  ADCSRA = 0b10011111; // ADC enable, manual trigger mode, ADC interrupt enable, prescaler = 128
  
  // Set up a timer 2 interrupt every 1ms to kick off ADC conversions etc.
  // Do this last after we have initialized everything else
  ASSR = 0;
  TCCR2A = (1 << WGM21);    // CTC mode
  TCCR2B = (1 << CS22);     // prescaler = 64 (256? -> 4ms(250Hz))
  TCNT2 = 0;                // restart counter
  OCR2A = 249;              // compare register = 249, will be reached after 1ms (1000Hz)
  TIMSK2 = (1 << OCIE2A);
}

void setup() {

  #ifdef testmode
  Serial.begin(115200);
  Serial.println("Start");
  Serial.flush();
  #endif

  EEPROM_restoreValues();
  timer.setInterval(1000L*3600L, EEPROM_storeValues); //once in 1h remember critical values

  //analogReference(INTERNAL); //1.1 on 328p
  analogReference(EXTERNAL); //connected to 3.3V pin
  startTime = millis();

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
		#ifdef testmode
		Serial.println("Error Initializing CAN bus driver MCP2515...");
		#endif
	}

  //initialize filters Masks(0-1),Filters(0-5):
  unsigned long mask = (0x0100L | CAN_Unit_MASK)<<16;			//0x0F	0x010F0000;
  unsigned long filt = (0x0100L | CAN_Unit_FILTER_BatMo)<<16;	//0x04	0x01040000;
  //first mask:   ID=0x100 - receive sent to all 0x100
  CAN0.init_Mask(0,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
  CAN0.init_Filt(1,0,0x01000000);                // Init second filter...
  //second mask:  ID=0x010F - receive only sent to our unit
  CAN0.init_Mask(1,0,mask);                // Init second mask...
  CAN0.init_Filt(2,0,filt);                // Init third filter...
  CAN0.init_Filt(3,0,filt);                // Init fouth filter...
  CAN0.init_Filt(4,0,filt);                // Init fifth filter...
  CAN0.init_Filt(5,0,filt);                // Init sixth filter...

  //#ifdef testmode
	//CAN0.setMode(MCP_LOOPBACK);
	//#endif
	//#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
	//#endif
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input

  //pinMode(13,OUTPUT);//led

  mainTimerId = timer.setInterval(1000L*MainCycleInterval, MainCycle_StartEvent); //start regularly
  //timer.setInterval(300L,measureVoltage);

  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,57497.1); //START

  SumDeltaWs = 0;
  setupADC_and_TimerInt();
}

void loop() {
  timer.run();
  checkReadCAN();
}


ISR(TIMER2_COMPA_vect){ // Interrupt service routine for the 1ms tick
  TIFR2 = (1 << OCF2B);  // shouldn't be needed according to the documentation, but is (perhaps the ISR is too short without it?)
  // Kick off a new ADC conversion. We already set the multiplexer to the correct channel when the last conversion finished.
  ADCSRA = 0b11001111;   // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128
}

ISR(ADC_vect){ // Interrupt service routine for ADC conversion complete
  // The mcu requires us to read the ADC data in the order (low byte, high byte)
  unsigned char adcl = ADCL;
  unsigned char adch = ADCH;
  unsigned int adcVal = (adch << 8) | adcl; //current ADC reading
  
  switch(currentAdcChannel){ 
    case MonitorVPin:
      currentAdcChannel = MonitorAPin;

      voltageV = (float)adcVal*voltageCoef; //momentary value V

      //sliding average V:
      //VArSumTotal = VArSumTotal - VArReadings[VArIndex] + voltageV; 
      VArReadings[VArIndex] = voltageV; 
      VArIndex++;
      if (VArIndex >= VArNumReadings){             
        VArIndex = 0;                           
      }
      //AverageV = VArSumTotal / VArNumReadings; 

    break;
    case MonitorAPin:
      currentAdcChannel = MonitorVPin;

      int curVal = adcVal-curOffsetMidpoint;
      if(-1<=curVal && curVal<=1){ //cut off noise
        curVal=0;
      }
      currentA = (float)(curVal)*currentCoef;
      float DeltaWs = voltageV*currentA*deltaTms/1000.0f;
      if(DeltaWs>0.0f){ //when charging - there is loss - cause we measure IN but calibrate by OUT energy, which we can get!
        DeltaWs /= 1.065f; //13.3/12.4 - about 7% loss on charging - energy stored is accounted as energy you can get out of it!
      }
      SumDeltaWs += DeltaWs;

    break;
  }
  
  // Set the ADC multiplexer to the channel we want to read next. Setting it here rather than in the tick ISR allows the input
  // to settle, which is required for ADC input sources with >10k resistance.
  ADMUX = (0b00000000 | currentAdcChannel);   // ext reference, select current channel
}

