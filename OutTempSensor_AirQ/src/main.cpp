//#if 1
///OutTempSensor///
//For DS18B20 connected to separate pins (no need to know adresses, means interchangebility):

//#define testmode

#include <OneWire.h>

#define CAN_PIN_INT 9      // INT = pin 9
#define CAN_PIN_CS 10      // CS = pin 10 

#include <NIK_defs.h>
#include <NIK_can.h>

#define LED_PIN 13

OneWire  TempDS_Outdoor(4); //3 new board nano // on pin 2 (Uno)(a 4.7K resistor to 5V is necessary)

int eepromVIAddr=1000,eepromValueIs=7890+3; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=60; //5 это тестирование датчиков, а ставим не чаще 60
float tempOutdoor=0;
float tempOutdoor_calibrationOffset=0.5;

int mainTimerId;

void TempDS_AllStartConvertion() {
  TempDS_Outdoor.reset();
  TempDS_Outdoor.write(0xCC); //skip rom - next command to all //ds.select(addr);
  TempDS_Outdoor.write(0x44); // start conversion
}

byte TempDS_GetTemp(OneWire *ds, String dname, float *temp) { //interface object and sensor name, returns 1 if OK
  byte data[12];

  ds->reset(); //перезагрузка с отключением питания (делаем перед запросом на замер, тут не надо)
  ds->write(0xCC); //Skip ROM - next command to all //ds.select(addr);
  ds->write(0xBE); //Read Scratchpad

  //Serial.print(dname);
  //Serial.print(" :");
  for (byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds->read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  if(data[8]!=OneWire::crc8(data, 8)) {
	  Serial.println("");
	  Serial.print("!ERROR: temp sensor CRC failure - ");
	  Serial.println(dname);
	  return 0; //crc failure
  }

  // Calculate temperature value
  *temp = (float)( (data[1] << 8) + data[0] )*0.0625F;

  #ifdef testmode
  Serial.print(" t=");
  Serial.print(*temp);
  #endif

  return 1; //OK
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
	switch(vPinNumber){
		case VPIN_STATUS:
			boardSTATUS = (int)vPinValueFloat;
      EEPROM_storeValues();
			break;
		//case VPIN_SetMainCycleInterval:
		//	if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<5)
		//		break;
		//	MainCycleInterval = (int)vPinValueFloat;
		//	timer.deleteTimer(mainTimerId);
		//	mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
		//	break;
		//case VPIN_ManualFloorIn:
		//	tempTargetFloorIn = vPinValueFloat;
		//	break;
		default:
      #ifdef testmode
			Serial.print("! Warning: received unneeded CAN message: VPIN=");
			Serial.print(vPinNumber);
			Serial.print(" FloatValue=");
			Serial.print(vPinValueFloat);
			Serial.println();
      #endif
			return 0;
	}
	return 1;
}

void EEPROM_storeValues(){
  return;
  EEPROM_WriteAnything(eepromVIAddr,eepromValueIs);
  EEPROM_WriteAnything(VPIN_STATUS, (unsigned char)boardSTATUS);
  EEPROM_WriteAnything(VPIN_MainCycleInterval, (unsigned char)(MainCycleInterval/10));
  //EEPROM.update(VPIN_STATUS,(unsigned char)boardSTATUS);
  //EEPROM.update(VPIN_MainCycleInterval,(unsigned char)(MainCycleInterval/10));
}
void EEPROM_restoreValues(){
  return;
  int ival;
  EEPROM_ReadAnything(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }

  unsigned char val;
  EEPROM_ReadAnything(VPIN_STATUS,val);
  boardSTATUS = val;
  EEPROM_ReadAnything(VPIN_MainCycleInterval,val);
  if(val==0)
    val=6;
  MainCycleInterval = val*10;
}

//////////////////////////////////////////////////////////////////MAIN cycle:
void MainCycle_ReadTempEvent(){

  int ErrorMeasTemp = 0;
  if (!TempDS_GetTemp(&TempDS_Outdoor, "OUTDOOR", &tempOutdoor))
  {
    ErrorMeasTemp++;
  }
  #ifdef testmode
  Serial.println();
  #endif

  if (!ErrorMeasTemp)
  {
    tempOutdoor += tempOutdoor_calibrationOffset;
    //Serial.println(tempOutdoor);
    tempOutdoor = (float)((long)((tempOutdoor + 0.05) * 10)) / 10.0;
    //Serial.print("sending ");
    //Serial.println(tempOutdoor);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorTemp, tempOutdoor);
  }else{
    #ifdef testmode
    Serial.print("Error measuring temp. ErrorMeasTemp = ");
    Serial.println(ErrorMeasTemp);
    #endif
  }
}

void MainCycle_StartEvent(){
  switch (boardSTATUS)
  {
		case Status_Standby:
			return;	//skip standby state
	}

	TempDS_AllStartConvertion();
  #ifdef testmode
	Serial.println("Start coversion... ");
  #endif

	timer.setTimeout(1000L, MainCycle_ReadTempEvent); //start once after timeout
}
////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
  Serial.begin(115200);

  //EEPROM_storeValues();
  EEPROM_restoreValues();
  //timer.setInterval(1000L*600L, EEPROM_storeValues); //once in 10 min remember critical values
    
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STDEXT - they are the only working modes
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing CAN bus driver MCP2515...");
  
  //initialize filters Masks(0-1),Filters(0-5):
  //unsigned long mask  = (0x0100L | CAN_Unit_MASK | CAN_MSG_MASK)<<16;			//0x0F	0x010F0000;
  //unsigned long filt0 = (0x0100L | CAN_Unit_FILTER_OUTDT | CAN_MSG_FILTER_UNITCMD)<<16;	//0x04	0x01040000;
  //unsigned long filt1 = (0x0100L | CAN_Unit_FILTER_OUTDT | CAN_MSG_FILTER_INF)<<16;	//0x04	0x01040000;
  //CAN0.init_Mask(0,0,mask);                // Init first mask...
  //CAN0.init_Filt(0,0,filt0);                // Init first filter...
  //#ifdef testmode
  //CAN0.init_Filt(1,0,filt1);                // Init second filter...
  //#endif
  //CAN0.init_Mask(1,0,0x01FFFFFF);                // Init second mask...
  //CAN0.init_Filt(2,0,0x01FFFFFF);                // Init third filter...
  //CAN0.init_Filt(3,0,filt);                // Init fouth filter...
  //CAN0.init_Filt(4,0,filt);                // Init fifth filter...
  //CAN0.init_Filt(5,0,filt);                // Init sixth filter...

  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
  #ifdef testmode
  CAN0.setMode(MCP_LOOPBACK);
  #endif
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input

  mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
}

////////////////////////////////////////////////LOOP////////////////////////////
void loop(void) {
  timer.run();
  checkReadCAN();
}

//#endif
