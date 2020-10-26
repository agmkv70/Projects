//#if 1
///OutTempSensor///
//For DS18B20 connected to separate pins (no need to know adresses, means interchangebility):

//#define testmode

#include <OneWire.h>
#include <SoftwareSerial.h> //for airQuality censor

#define CAN_PIN_INT 9      // INT = pin 9
#define CAN_PIN_CS 10      // CS = pin 10 

#include <NIK_defs.h>
#include <NIK_can.h>

#define LED_PIN 13

OneWire  TempDS_Outdoor(4); //3 new board nano // on pin 2 (Uno)(a 4.7K resistor to 5V is necessary)
SoftwareSerial AirQSerial(6, 5); //2 digital pins
unsigned char AirQRead_buf[30],AirQBufCounter=0,AirQMesAvailFlag=0,AirQReadBufAndSendFlag=0;
//for defining outstanding values (noise):
float Temperature1=0,Humidity1=0,Temperature2=0,Humidity2=0;
float Gas1=0,Gas2=0;
float Pressure1=0,Pressure2=0;
uint16_t IAQ1=0,IAQ2=0;

int eepromVIAddr=1000,eepromValueIs=7890+3; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=30; //5 это тестирование датчиков, а ставим не чаще 60
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
    tempOutdoor = fround(tempOutdoor,1);
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

char isValidSeries3(float v1,float v2,float v3,float dev){ //v3 difference is not than dev
  float dif = v3-v1;
  if(-dev<=dif && dif<=dev){
    return 1;
  }
  dif = v2-v1;
  if(-dev<=dif && dif<=dev){
    return 2;
  }
  return 0;
}

/////////////////////////////////AirQuality functions:
void TurnOffLED(){
  digitalWrite(LED_PIN, LOW);
}

void SendAirQInfo(){
  float Temperature,Humidity;
  unsigned char i=0,sum=0;
  uint32_t Gas;
  uint32_t Pressure;
  uint16_t IAQ;
  //int16_t  Altitude;
  uint8_t  IAQ_accuracy;

  if(AirQMesAvailFlag){ //there is a message in the buffer
    AirQMesAvailFlag=0;

    if(AirQRead_buf[0]==0x5A && AirQRead_buf[1]==0x5A ){ //check buffer start again (not needed really)   
      for(i=0;i<19;i++){
        sum+=AirQRead_buf[i]; 
      }
      if(sum==AirQRead_buf[i] ){ //checksum
        //here we have full buffer with consistent message - extract and send to CAN bus:
        digitalWrite(LED_PIN, LOW); //show sending AirQ
        timer.setTimeout(100, TurnOffLED);
        
        int16_t temp2=(AirQRead_buf[4]<<8|AirQRead_buf[5]);   
        Temperature=(float)temp2/100;
        uint16_t temp1=(AirQRead_buf[6]<<8|AirQRead_buf[7]);
        Humidity=(float)temp1/100; 
        Pressure=((uint32_t)AirQRead_buf[8]<<16)|((uint16_t)AirQRead_buf[9]<<8)|AirQRead_buf[10];
        IAQ_accuracy= (AirQRead_buf[11]&0xf0)>>4;
        IAQ=((AirQRead_buf[11]&0x0F)<<8)|AirQRead_buf[12];
        Gas=((uint32_t)AirQRead_buf[13]<<24)|((uint32_t)AirQRead_buf[14]<<16)|((uint16_t)AirQRead_buf[15]<<8)|AirQRead_buf[16];
        //Altitude=(AirQRead_buf[17]<<8)|AirQRead_buf[18]; 

        CANQueueStop=1;
        if(isValidSeries3(Temperature1,Temperature2,Temperature,1) && Temperature>=-50 && Temperature<=60){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_Temperature, fround(Temperature,1));
        }
        if(isValidSeries3(Humidity1,Humidity2,Humidity,3) && Humidity>=0 && Humidity<=100){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_Humidity, fround(Humidity,0));
        }
        float dPressure = Pressure/1000; //kPa: 100kPa = 10^5 Pa
        if(isValidSeries3(Pressure1,Pressure2,dPressure,0.05) && dPressure>=80 && dPressure<=120){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_Pressure, dPressure);
        }
        float dGas=Gas/1000;
        if(isValidSeries3(Gas1,Gas2,dGas,5) && dGas>=0 && dGas<=300){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_Gas, fround(dGas,1));
        }
        //addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_Altitude, Altitude);
        if(isValidSeries3(IAQ1,IAQ2,IAQ,50)){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_IAQ, IAQ);
        }
        if(IAQ_accuracy>=0 && IAQ_accuracy<=3){
          addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_OutdoorAirQ_IAQ_accuracy, IAQ_accuracy);
        }
        CANQueueStop=0;

        Temperature1=Temperature2;
        Temperature2=Temperature;
        Humidity1=Humidity2;
        Humidity2=Humidity;
        Pressure1=Pressure2;
        Pressure2=dPressure;
        Gas1=Gas2;
        Gas2=dGas;
        IAQ1=IAQ2;
        IAQ2=IAQ;
        //Temperature:26.54 ,Humidity:39.26 ,Pressure:100574 ,Gas:28312  ,Altitude:62  ,IAQ:25  ,IAQ_accuracy:0
        /*Serial.print("Temperature:");Serial.print(Temperature); 
        Serial.print(" ,Humidity:");Serial.print(Humidity); 
        Serial.print(" ,Pressure:");Serial.print(Pressure);     
        Serial.print(" ,Gas:");Serial.print(Gas ); 
        Serial.print("  ,Altitude:");Serial.print(Altitude);                       
        Serial.print("  ,IAQ:");Serial.print(IAQ); 
        Serial.print("  ,IAQ_accuracy:");Serial.println(IAQ_accuracy); //0-starting; 1-too stable env; 2-(re)calibrating; 3-working
        */
      }            
    }
  } 
}

void AirQCheckSerialAndSend(){
  //check for AirQ message:
  while (AirQMesAvailFlag==0 && AirQSerial.available()){   
    AirQRead_buf[AirQBufCounter] = (unsigned char)AirQSerial.read();
    
    if(AirQBufCounter==0 && AirQRead_buf[0]!=0x5A) 
      return;         
    if(AirQBufCounter==1 && AirQRead_buf[1]!=0x5A){
      AirQBufCounter=0;
      return;
    }           
    AirQBufCounter++;       
    if(AirQBufCounter==20){    
      AirQBufCounter=0;                  
      AirQMesAvailFlag=1;
    }      
  }
  if(AirQMesAvailFlag){
    SendAirQInfo();
    //AirQMesAvailFlag=0; //its done inside
    AirQReadBufAndSendFlag=0;
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

  AirQReadBufAndSendFlag=1; //have to listen to softSerial and send one first message we've read

	timer.setTimeout(1000L, MainCycle_ReadTempEvent); //start once after timeout
}
////////////////////////////////////////////////SETUP///////////////////////////
void setup(void) {
  delay(500);
  #ifdef testmode
  Serial.begin(115200);
  #endif

  //EEPROM_storeValues();
  //EEPROM_restoreValues();
  //timer.setInterval(1000L*600L, EEPROM_storeValues); //once in 10 min remember critical values
    
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW); //turn off LED

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STDEXT - they are the only working modes
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else{
    #ifdef testmode
    Serial.println("Error Initializing CAN bus driver MCP2515...");
    #endif
  }
  
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

  //////////////////////AirQ communication <->STM<->BME680 :
  AirQSerial.begin(9600); //9600 is programmed into STM
  AirQSerial.listen();  
  delay(4000);    
  AirQSerial.write(0XA5); 
  AirQSerial.write(0X55);    
  AirQSerial.write(0X3F);    
  AirQSerial.write(0X39); 
  delay(100); 
  AirQSerial.write(0XA5); 
  AirQSerial.write(0X56);    
  AirQSerial.write(0X02);    
  AirQSerial.write(0XFD);

  mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
}

////////////////////////////////////////////////LOOP////////////////////////////
void loop(void) {
  timer.run();
  checkReadCAN();

  if(AirQReadBufAndSendFlag){
    AirQCheckSerialAndSend();
  }
}

//#endif
