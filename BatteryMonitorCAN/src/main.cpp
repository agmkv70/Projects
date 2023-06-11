//#define testmode

#define CAN_PIN_INT 9
#define CAN_PIN_CS  10 

#include <NIK_defs.h>
#include <NIK_can.h>

#include <avr/io.h>
#include <avr/interrupt.h>

const int MonitorVPin = 0;
const int MonitorAPin = 1;
static volatile unsigned char currentAdcChannel;

#define PIN_MEASUREVOLT A0

float voltage=0, voltageCoef=0.01599386f; // divider- 51:680 -maxV`15.75
const int VArNumReadings = 10;
float VArReadings[VArNumReadings]; //array for sliding average
int VArIndex = 0;                   //current index
float VArSumTotal = 0;
long startTime;                

int mainTimerId;
int eepromVIAddr=1000,eepromValueIs=7650+0; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=30; //seconds

void OnVoltageMeasured();

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
		Serial.println(voltage);
  #endif

  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,fround(voltage,2)); //rounded 0.0 value
}

void MainCycle_StartEvent(){
  //digitalWrite(13,HIGH);

  //float avolt = voltage;
    
  #ifdef testmode
		Serial.print("----------a0volt = ");
		Serial.println(fround(voltage,2));
  #endif

  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_LEDPower12Voltage,fround(avolt,2)); //rounded 0.0 value
  addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_DCVoltmeter1,fround(voltage,2));
  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,1000000+fround(voltage,2));
  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_Floor_ECHO,111); 

	//digitalWrite(13,LOW);
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
		case VPIN_LEDSetMainCycleInterval:
			if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
				break;
			MainCycleInterval = (int)vPinValueFloat;
			timer.deleteTimer(mainTimerId);
			mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
			EEPROM_storeValues();
			break;
		//case VPIN_LEDSetPWMch1:  PWMch1  = vPinValueFloat; analogWrite(PWMpin1,PWMch1); EEPROM_storeValues(); break;
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
  
  EEPROM.update(VPIN_STATUS,(unsigned char)boardSTATUS);
  EEPROM.update(VPIN_LEDMainCycleInterval,(unsigned char)(MainCycleInterval/10));

  //EEPROM.update(VPIN_LEDSetPWMch1,(unsigned char)PWMch1);
  //EEPROM.update(VPIN_LEDSetPWMch2,(unsigned char)PWMch2);
  //EEPROM.update(VPIN_LEDSetPWMch3,(unsigned char)PWMch3);
  //EEPROM.update(VPIN_LEDSetPWMch4,(unsigned char)PWMch4);

}
void EEPROM_restoreValues(){
  int ival;
  EEPROM_ReadAnything(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }

  boardSTATUS = EEPROM.read(VPIN_STATUS);
  
  int aNewInterval = EEPROM.read(VPIN_LEDMainCycleInterval)*10;
  if(aNewInterval != 0){
    MainCycleInterval = aNewInterval;
  }
  
  //PWMch1 = EEPROM.read(VPIN_LEDSetPWMch1);
  //PWMch2 = EEPROM.read(VPIN_LEDSetPWMch2);
  //PWMch3 = EEPROM.read(VPIN_LEDSetPWMch3);
  //PWMch4 = EEPROM.read(VPIN_LEDSetPWMch4);
}

void setupADC_and_TimerInt(){
   // Set up the ADC. We need to read the mains voltage frequently in order to get a sufficiently accurate RMS reading.
  // To avoid using too much CPU time, we use the conversion-complete interrupt. This means we can't use analogRead(),
  // we have to access the ADC ports directly. 
  currentAdcChannel = MonitorVPin;      // set up which analog input we will read first
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

void setup() {

  #ifdef testmode
  Serial.begin(9600);
  Serial.println("Start");
  Serial.flush();
  #endif

  //EEPROM_restoreValues();
  //timer.setInterval(1000L*600L, EEPROM_storeValues); //once in 10 min remember critical values

  analogReference(INTERNAL); //1.1 on 328p
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
  timer.setInterval(300L,measureVoltage);

  //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,57497.1); //START

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
      //...

      currentAdcChannel = MonitorAPin;
    break;
    case MonitorAPin:
      //...

      currentAdcChannel = MonitorVPin;
    break;
  }
  
  // Set the ADC multiplexer to the channel we want to read next. Setting it here rather than in the tick ISR allows the input
  // to settle, which is required for ADC input sources with >10k resistance.
  ADMUX = (0b01000000 | currentAdcChannel);   // Vcc reference, select current channel
}

