#define testmode

#define CAN_PIN_INT 9
#define CAN_PIN_CS  10 

#include <NIK_defs.h>
#include <NIK_can.h>

#define PIN_MEASUREVOLT A0

float voltage=0, voltageCoef=0.015157616f;
const int VArNumReadings = 10;
float VArReadings[VArNumReadings]; //array for sliding average
int VArIndex = 0;                   //current index
float VArSumTotal = 0;
long startTime;                

int mainTimerId;
int eepromVIAddr=1000,eepromValueIs=7650+0; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=3; //seconds

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
  addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_LEDPower12Voltage,fround(voltage,2));
  addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,1000000+fround(voltage,2));
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


void setup() {

  #ifdef testmode
  Serial.begin(115200);
  Serial.println("Start");
  Serial.flush();
  #endif

  //EEPROM_restoreValues();
  //timer.setInterval(1000L*600L, EEPROM_storeValues); //once in 10 min remember critical values

  analogReference(INTERNAL);
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

  addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BLYNK_TERMINAL,57497.1); //START
}

void loop() {
  timer.run();
  checkReadCAN();
}

