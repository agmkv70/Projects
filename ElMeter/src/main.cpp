#define _MAX_FIXEDARRAY_DEFINED 7
#include <Arduino.h>
#include <SoftwareSerial.h>

#define CAN_PIN_INT   9    
#define CAN_PIN_CS   10 
#include <NIK_defs.h>
#include <NIK_can.h>

//#define HardSerial
#define testmodeS2 //was
//#define testmodeS
#define testmode //was
//#define testmodeCAN

//-------- CAN to meter
#ifndef HardSerial
#define SSerial_Rx 2     // was 0
#define SSerial_Tx 3     // was 1
SoftwareSerial ElCANSerial(SSerial_Rx, SSerial_Tx); // Rx, Tx
#endif

//    команды Меркурий 230:
//    |Адрес счетчика 1 байт | Запрос 1 байт|
volatile byte address[] = {232};// адрес мой 232

volatile byte test_cmd[] = {0}; // тестирование канала связи
volatile byte openUser_cmd1[] = {1,1,1,1,1,1,1,1}; // тестирование канала связи (code=1,level=1,pw=111111(hex))
volatile byte openUser_cmd2[] = {1,2,2,2,2,2,2,2}; // тестирование канала связи (code=1,level=2,pw=222222(hex))

volatile byte getTime_cmd[] = {4,0};    // read=4, cur.time=0
volatile byte setTime_cmd[] = {3,0x0C, 0,0,0,0,0,0,0,0}; // ss,mm,hh,wd,MM,YY,(s=0/w=1)
volatile byte setTimeCorr_cmd[] = {3,0x0D, 0,0,0};       // ss,mm,hh +-4min/day

volatile byte getEnergy_cmd[] = {5,0x00,0};    // readEnergy=5, from=0+month=0, tarif=0(sum=0)
volatile byte curent_PSum_cmd[] = {8,0x16,0x00}; //phase sum=0x00, 0x01-1phase,...
//byte curent_P1_cmd[] = {8,0x14,0x01}; //Power 1 phase
//byte curent_P2_cmd[] = {8,0x14,0x02};
//byte curent_P3_cmd[] = {8,0x14,0x03};
/*byte curent_I1_cmd[] = {8,0x14,0x21}; //Current 1 phase
byte curent_I2_cmd[] = {8,0x14,0x22};
byte curent_I3_cmd[] = {8,0x14,0x23};*/
volatile byte curent_Uall_cmd[] = {8,0x16,0x11}; //U all phase?
//byte curent_U1_cmd[] = {8,0x14,0x11}; // U 1 phase
//byte curent_U2_cmd[] = {8,0x14,0x12};
//byte curent_U3_cmd[] = {8,0x14,0x13};

//#define KWhDeltaIntervalmillis 3600000L;
volatile unsigned long pred_EnergyKWhmillis=0;
volatile float _pred_EnergyKWh=0, _corr_EnergyKWh=46400.16; //to return delta//to know main meter value (initval=22/06/2022)
volatile float _corr_EnergyKWh1=0,_corr_EnergyKWh2=0;
#define MAXRESPONSE 21
volatile byte response[MAXRESPONSE+4]; // длина массива входящего сообщения
volatile byte address_cmd_crc[MAXRESPONSE+4];
volatile int byteReceived;
volatile int byteSend;
//unsigned long time_sent_kWh=0, time_sent_kWhDelta=0;

unsigned int crc16MODBUS(const byte *nData, int count);
void Send2ServerElMeterData();
#ifdef HardSerial
  void SerialCleanSwap(){
    //clear read buffer
    while(Serial.available()){
      Serial.read();
    }
    Serial.flush(); //write everything
    Serial.swap();
  }
#endif //HardSerial

///////////////////////////////////////////////////
void setup(){ 
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN,1);
  delay(500);
  digitalWrite(LED_BUILTIN,0);
  delay(300);
  
  #ifdef testmodeS
    //Serial.begin(115200);
    //Serial.begin(9600);
  #endif //testmodeS
  #ifdef testmodeS
    Serial.println("***setup start***");
    delay(100);//Serial.flush();
  #endif //testmodeS

  //byte res = ElMeter_SetTimeCorr(20,05,00);
  //Serial.print("SetTime: ");
  //Serial.println(res);

  /////////////////////////////////////////////////////////////////////////////////////
  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK){ //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  }else{  
    #ifdef testmodeS
    Serial.println("Error Initializing CAN bus driver MCP2515...");
    #endif
  }

  //initialize filters Masks(0-1),Filters(0-5):
  // unsigned long mask  = (0x0100L | CAN_Unit_MASK | CAN_MSG_MASK)<<16;      //0x0F  0x010F0000;
  // unsigned long filt0 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_UNITCMD)<<16;  //0x04  0x01040000;
  // unsigned long filt1 = (0x0100L | CAN_Unit_FILTER_KUHFL | CAN_MSG_FILTER_INF)<<16;  //0x04  0x01040000;
  //receive 0x100 messages:
  CAN0.init_Mask(0,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
  CAN0.init_Filt(1,0,0x01000000);

  CAN0.init_Mask(1,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(2,0,0x01000000);
  CAN0.init_Filt(3,0,0x01000000);
  CAN0.init_Filt(4,0,0x01000000);
  CAN0.init_Filt(5,0,0x01000000);
  
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input
  
  #ifndef HardSerial
    //softserial
    ElCANSerial.begin(9600);
    if (!ElCANSerial) { // If the object did not initialize, then its configuration is invalid
      Serial.println("!!Can't connect to ElMeter!");// Probably invalid SoftwareSerial pin configuration!!"); 
      int on=1;
      while (1) { // Don't continue with invalid configuration
        digitalWrite(LED_BUILTIN,on);
        delay (300);
        on^=1;
      }
    } 
  #endif //softserial
  
  /*for(int i=0;i<timer.getNumTimers();){
    timer.deleteTimer(i);
  }*/
  timer.setInterval(3000L, Send2ServerElMeterData); 
  #ifdef testmodeS
    Serial.println("***setup end***");
    //Serial.flush();
  #endif //testmodeS
  delay(100);//

  /*int i=0,on=1;
   while (i<10) { // Don't continue with invalid configuration
        digitalWrite(LED_BUILTIN,on);
        delay (300);
        on^=1;
        i++;
      }*/
  
}

void loop(){
  /*delay(1000);
  Serial.println("loop()");
  Serial.flush();
  */
  //loop CANbus+Blynk
  timer.run();
  checkReadCAN();
  //////////////////////////////
  /*return;
  
  //#ifdef testmodeS
    Serial.println();

    Serial.print("Test connection: ");
    byte res=0;
    res = ElMeter_TestConnection();
    Serial.println(res);
    if(res<=0){
      ElCANSerial.begin(9600);
      return;
    } 

    Serial.print("Login user1: ");
    res = ElMeter_OpenUser(1);
    Serial.println(res);

    byte YY,MM,DD,hh,mm,ss,sumerWinter;
    Serial.print("Get time: ");
    res = ElMeter_GetTime(&YY,&MM,&DD,&hh,&mm,&ss,&sumerWinter);
    Serial.print(res);
    Serial.print(" ");
    Serial.print(YY);
    Serial.print("-");
    Serial.print(MM);
    Serial.print("-");
    Serial.print(DD);
    Serial.print(" ");
    Serial.print(hh);
    Serial.print(":");
    Serial.print(mm);
    Serial.print(":");
    Serial.print(ss);
    Serial.println();
    
    float Wh;
    Serial.print("Wh: ");
    res = ElMeter_GetEnergyA(&Wh,0);
    Serial.print(res,3);
    Serial.print(" = ");
    Serial.println(Wh,3);
    res = ElMeter_GetEnergyA(&Wh,1);
    Serial.print("Wh1: ");
    Serial.print(res);
    Serial.print(" = ");
    Serial.println(Wh,3);
    res = ElMeter_GetEnergyA(&Wh,2);
    Serial.print("Wh2: ");
    Serial.print(res);
    Serial.print(" = ");
    Serial.println(Wh,3);
    
    float ph1,ph2,ph3;
    Serial.print("Power: ");
    res = ElMeter_GetInstantPower(&ph1,&ph2,&ph3);
    Serial.print(res);
    Serial.print("; 1= ");
    Serial.print(ph1,3);
    Serial.print(" 2= ");
    Serial.print(ph2,3);
    Serial.print(" 3= ");
    Serial.println(ph3,3);
    
    Serial.print("Voltage: ");
    res = ElMeter_GetInstantVoltage(&ph1,&ph2,&ph3);
    Serial.print(res);
    Serial.print("; 1= ");
    Serial.print(ph1);
    Serial.print(" 2= ");
    Serial.print(ph2);
    Serial.print(" 3= ");
    Serial.println(ph3);
*/
    /*
    ElMeter__ExecQuery(test_cmd, sizeof(test_cmd), 4);
    ElMeter__ExecQuery(openUser_cmd1, sizeof(openUser_cmd1), 4);
    ElMeter__ExecQuery(getTime_cmd, sizeof(getTime_cmd), 11); //(adr) 03 28 21,01,31,01 22,01 (CRC): 21:28:03,Monday,31,jan,2022y,winter(=1)
    //Ответ: (80) 43 14 16 03 27 02 08 01 (CRC).
    //Результат: 16:14:43 среда 27 февраля 2008 года, зима
    ElMeter__ExecQuery(getEnergy_cmd, sizeof(getEnergy_cmd), 19); //4x4 bytes (A+,A-,R+,R-)
    ElMeter__ExecQuery(curent_PSum_cmd, sizeof(curent_PSum_cmd), 15); //4x3 bytes (sum,ph1,ph2,ph3)
    ElMeter__ExecQuery(curent_Uall_cmd, sizeof(curent_Uall_cmd), 12); //3x3 (ph1,ph2,ph3)
    */
    /*
    Send HEX:  e8 0 4f b0
    Received:  e8 0 4f b0  - 11 : 4 < 4
    Send HEX:  e8 1 1 1 1 1 1 1 1 d9 85
    Received:  e8 0 4f b0  - 12 : 4 < 4
    Send HEX:  e8 4 0 f3 34
    Received:  e8 36 5 16 3 16 2 22 1 48 d4  - 21 : 11 < 11
    Send HEX:  e8 5 0 0 25 85
    Received:  e8 68 1 59 10 ff ff ff ff 3 0 63 4d ff ff ff ff 47 7a  - 36 : 19 < 19
    Send HEX:  e8 8 16 0 ba 26
    Received:  e8 80 22 0 0 0 0 80 22 0 0 0 0 27 3  - 24 : 15 < 15
    Send HEX:  e8 8 16 11 7a 2a 
    Received:  e8 0 0 0 0 75 58 0 0 0 6b 10  - 19 : 12 < 12
    */

    //delay(8000);
  
  //#endif //testmodeS
}

unsigned int crc16MODBUS(volatile byte *nData, int count){ // Расчет контрольной суммы для запроса
  volatile static unsigned int crcTable[] = {
  0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
  0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
  0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
  0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
  0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
  0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
  0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
  0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
  0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
  0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
  0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
  0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
  0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
  0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
  0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
  0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
  0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
  0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
  0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
  0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
  0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
  0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
  0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
  0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
  0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
  0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
  0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
  0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
  0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
  0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
  0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
  0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

  volatile unsigned int crc = 0xFFFF;
  for (int i = 0; i < count; i++){
    crc = ((crc >> 8) ^ crcTable[(crc ^ nData[i]) & 0xFF]);
  }
  
  //#ifdef testmodeS2
    //Serial.print(" crc=");
    //Serial.println(crc);
    //Serial.flush();
  //#endif
  return crc;

  /*byte nTemp;
  unsigned int wCRCWord = 0xFFFF;
  while (count--){
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= crcTable[nTemp];
  }
  return wCRCWord;*/
}

byte ElMeter__ExecQuery(volatile byte *cmd, int s_cmd, byte responseLength){ // для тестовых запросов {Адрес счетчика 4, 	Запрос 1, 	CRC16 (Modbus) 2}
  int s_addr = 1;
  int s_address_cmd = s_cmd + s_addr; //plus address byte
  int s_address_cmd_crc = s_address_cmd + 2; //plus CRC

  #ifdef testmodeS
    Serial.print("ExecQ: ");
    Serial.flush();
  #endif
  //write adress:
  int pos = 0;
  for (int i = 0; i < s_addr; i++){
    address_cmd_crc[pos++] = address[i];
  }
  //add command:
  for (int i = 0; i < s_cmd; i++){
    address_cmd_crc[pos++] = cmd[i];
  }
  #ifdef testmodeS
    Serial.print("CRC: ");
    Serial.flush();
  #endif
  
  unsigned int crc = crc16MODBUS(address_cmd_crc, s_address_cmd);
  #ifdef testmodeS
    Serial.print("CRCOK: ");
    Serial.flush();
  #endif
  
  byte crc1 = crc & 0xFF;
  byte crc2 = (crc >> 8) & 0xFF;
  address_cmd_crc[pos++] = crc1;
  address_cmd_crc[pos] = crc2;

  #ifdef testmodeS
    Serial.print("Send HEX:  ");
    Serial.flush();
    //print command:
    String temp_term2 = "";
    for (int i = 0; i < s_address_cmd_crc; i++){
      temp_term2 += String(address_cmd_crc[i], HEX);
      temp_term2 += " ";
    }
    Serial.println(temp_term2);
    Serial.flush();
  #endif
  
  #ifdef HardSerial
    SerialCleanSwap();
  #else
    ElCANSerial.flush();
    while(ElCANSerial.available()){
      ElCANSerial.read();
    }
    Serial.flush();
  #endif

  //send:
  for (int i = 0; i < s_address_cmd_crc; i++){
    #ifdef HardSerial
      Serial.write(address_cmd_crc[i]);
    #else
      ElCANSerial.write(address_cmd_crc[i]);
    #endif
  }
  #ifdef HardSerial
    Serial.flush();
  #else
    ElCANSerial.flush();
    //while(ElCANSerial.available()){
    //  ElCANSerial.read();
    //}
  #endif
  delay(5);
  
  //receive:
  unsigned long receivetimeout=150, lastreadmillis;
  lastreadmillis = millis();
  byte irec = 0;
  do{ 
    //delay(5);
    #ifdef HardSerial
      while(Serial.available() && irec<MAXRESPONSE){
        byteReceived = Serial.read(); 
        response[irec++] = byteReceived;
        lastreadmillis = millis();
      }
    #else
      while(ElCANSerial.available() && irec < responseLength && irec < MAXRESPONSE){
        byteReceived = ElCANSerial.read();
        response[irec++] = byteReceived;
        lastreadmillis = millis();
      }
    #endif
  }while(irec < responseLength 
    && millis()-lastreadmillis < receivetimeout 
    && irec < MAXRESPONSE);
  
  #ifdef HardSerial
    SerialCleanSwap();
  #else
  #endif
  #ifdef HardSerial
    Serial.flush();
  #else
    ElCANSerial.flush();
    while(ElCANSerial.available()){
      ElCANSerial.read();
    }
  #endif
  

  //print received string:
  if(irec>0){
    #ifdef testmodeS
      Serial.print("Received:  ");
      String temp_term1 = "";
      for (unsigned int i = 0; i < irec; i++){
        temp_term1 += String(response[i], HEX);
        temp_term1 += " ";
      }
      Serial.print(temp_term1);
      //Serial.print(" - ");
      //Serial.print(millis()-startmillis);
      Serial.print(" : ");
      Serial.print(irec);
      Serial.print(" < ");
      Serial.print(//s_address_cmd_crc+
                    responseLength);
      Serial.println("");
    #endif
    if(irec==responseLength && responseLength>2){
      //check CRC
      crc = crc16MODBUS(response, irec-2);
      crc1 = crc & 0xFF;
      crc2 = (crc >> 8) & 0xFF;
      if(response[irec-2] == crc1 && response[irec-1] == crc2){
        return 1;//OK
      }else{
        #ifdef testmodeS
        Serial.print("Q: return -2;//crc error");
        #endif
        return -2;//crc error
      }
    }else{
      #ifdef testmodeS
      Serial.print("Q: return -1;//incorrect length");
      #endif
      return -1;//incorrect length
    }
  }else{
      #ifdef testmodeS
      Serial.print("Q: return 0;//no answear");
      #endif
      return 0;//no answear
  }
   
}

byte ElMeter_TestConnection(){
  byte res = ElMeter__ExecQuery(test_cmd, sizeof(test_cmd), 4);
  if(res>0){
    if(response[0]==address[0] && response[1]==0){
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_OpenUser(byte usr){
  byte res;
  if(usr==1){
    res = ElMeter__ExecQuery(openUser_cmd1, sizeof(openUser_cmd1), 4);
  }else if(usr==2){
    res = ElMeter__ExecQuery(openUser_cmd2, sizeof(openUser_cmd2), 4);
  }else{
    return -100; //wrong usr parameter
  }
  if(res>0){
    if(response[0]==address[0] && response[1]==0){
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte HD2Dec(byte x){
  return (x>>4)*10 + (x&0xF);
}
byte Dec2HD(byte x){
  return (x/10)<<4 | (x%10);
}
/*  
byte ElMeter_GetTime(byte *YY,byte *MM,byte *DD,byte *hh,byte *mm,byte *ss,byte *sumerWinter){
  byte res;
  res = ElMeter__ExecQuery(getTime_cmd, sizeof(getTime_cmd), 11);
  //Ответ: (80) 43 14 16 03 27 02 08 01 (CRC).
  //Результат: 16:14:43 среда 27 февраля 2008 года, зима
    
  if(res>0){
    if(response[0]==address[0]){
      *ss = HD2Dec(response[1]);
      *mm = HD2Dec(response[2]);
      *hh = HD2Dec(response[3]);
      *DD = HD2Dec(response[5]);
      *MM = HD2Dec(response[6]);
      *YY = HD2Dec(response[7]);
      *sumerWinter = HD2Dec(response[8]);
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_SetTime(byte YY,byte MM,byte DD,byte hh,byte mm,byte ss,byte sumerWinter){
  setTime_cmd[2] = Dec2HD(ss);
  setTime_cmd[3] = Dec2HD(mm);
  setTime_cmd[4] = Dec2HD(hh);
  setTime_cmd[6] = Dec2HD(DD);
  setTime_cmd[7] = Dec2HD(MM);
  setTime_cmd[8] = Dec2HD(YY);
  setTime_cmd[9] = sumerWinter;
  
  byte res;
  res = ElMeter__ExecQuery(setTime_cmd, sizeof(setTime_cmd), 4);
  //Ответ: (80) 43 14 16 03 27 02 08 01 (CRC).
  //Результат: 16:14:43 среда 27 февраля 2008 года, зима
    
  if(res>0){
    if(response[0]==address[0]){
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_SetTimeCorr(byte hh,byte mm,byte ss){
  setTime_cmd[2] = Dec2HD(ss);
  setTime_cmd[3] = Dec2HD(mm);
  setTime_cmd[4] = Dec2HD(hh);
  
  byte res;
  res = ElMeter__ExecQuery(setTimeCorr_cmd, sizeof(setTimeCorr_cmd), 4);
    
  if(res>0){
    if(response[0]==address[0]){
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}
*/
byte ElMeter_GetEnergyA(volatile float *ActiveWh,byte tariff){
  byte res;
  getEnergy_cmd[2]=tariff;
  res = ElMeter__ExecQuery(getEnergy_cmd, sizeof(getEnergy_cmd), 19); 
  //4x4 bytes (A+,A-,R+,R-)
  
  if(res>0){
    if(response[0]==address[0]){
      
      byte b1 = response[1];
      byte b2 = response[2];
      byte b3 = response[3];
      byte b4 = response[4];
      //b2_b1_b4_b3     68 1 59 10 > 01681059
      unsigned long _Active = (unsigned long)b2<<24 | (unsigned long)b1<<16 | (unsigned long)b4<<8 | (unsigned long)b3; 
      *ActiveWh = (float)_Active/1000;
      
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_GetInstantPower(volatile float *Ph1,volatile float *Ph2,volatile float *Ph3){
  byte res;
  res = ElMeter__ExecQuery(curent_PSum_cmd, sizeof(curent_PSum_cmd), 15); 
  //4x3 bytes (sum,ph1,ph2,ph3)
  
  if(res>0){
    if(response[0]==address[0]){
      
      byte b1 = response[4];
      byte b2 = response[5];
      byte b3 = response[6];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      unsigned long P = (unsigned long)(b1&0x3F)<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph1 = (float)P/100;
      
      b1 = response[7];
      b2 = response[8];
      b3 = response[9];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      P = (unsigned long)(b1&0x3F)<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph2 = (float)P/100;
      
      b1 = response[10];
      b2 = response[11];
      b3 = response[12];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      P = (unsigned long)(b1&0x3F)<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph3 = (float)P/100;
      
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_GetInstantVoltage(volatile float *Ph1,volatile float *Ph2,volatile float *Ph3){
  byte res;
  res = ElMeter__ExecQuery(curent_Uall_cmd, sizeof(curent_Uall_cmd), 12); 
  //3x3 (ph1,ph2,ph3)
  
  if(res>0){
    if(response[0]==address[0]){
      
      byte b1 = response[1];
      byte b2 = response[2];
      byte b3 = response[3];
      //b1_b3_b2
      unsigned long U = (unsigned long)b1<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph1 = (float)U/100;
      
      b1 = response[4];
      b2 = response[5];
      b3 = response[6];
      //b1_b3_b2
      U = (unsigned long)b1<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph2 = (float)U/100;
      
      b1 = response[7];
      b2 = response[8];
      b3 = response[9];
      //b1_b3_b2
      U = (unsigned long)b1<<16 | (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph3 = (float)U/100;
      
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

///////////////////////////////////////////////////

char ProcessReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
  // #ifdef testmodeS
  // Serial.print("received CAN message: VPIN=");
  // Serial.print(vPinNumber);
  // Serial.print(" FloatValue=");
  // Serial.print(vPinValueFloat);
  // Serial.println();
  // #endif
  switch(vPinNumber){
    //case VPIN_HEATER_TRGSTATUS:{
    //} break;
    case VPIN_ElMeter_EnergyKWh_cor:{
      _corr_EnergyKWh = vPinValueFloat;
    } break;
    case VPIN_ElMeter_EnergyKWh1_cor:{
      _corr_EnergyKWh1 = vPinValueFloat;
    } break;
    case VPIN_ElMeter_EnergyKWh2_cor:{
      _corr_EnergyKWh2 = vPinValueFloat;
    } break;
    default:{
      #ifdef testmodeS
      Serial.print("! Warning: received unneeded CAN message: VPIN=");
      Serial.print(vPinNumber);
      Serial.print(" FloatValue=");
      Serial.print(vPinValueFloat);
      Serial.println();
      #endif
      return 0;
    }
  }
  return 1;
}

////////////////////////////////////////////
void Send2ServerElMeterData(){
  volatile byte res = 0;
  volatile float EnergyKWh,EnergyKWh1,EnergyKWh2;
  volatile float P1,P2,P3;
  volatile float V1,V2,V3;
  
  digitalWrite(LED_BUILTIN,1);
  delay(500);
  digitalWrite(LED_BUILTIN,0);
  delay(500);
  
  #ifdef testmodeS2
    Serial.println("-Send2ServerElMeterData()-");
  #endif //testmodeS
  
  //res = ElMeter_TestConnection();
  /*if(res<=0){
    delay(300);
    ElCANSerial.begin(9600);
    delay(300);
    return;
  }*/

  res = ElMeter_OpenUser(1);
  #ifdef testmodeS2
    Serial.print("Open User1 = ");
    Serial.println(res);
  #endif //testmodeS

  res = ElMeter_GetEnergyA(&EnergyKWh,0);
  #ifdef testmodeS2
    Serial.print("Wh: ");
    Serial.print(res,3);
  #endif //testmodeS
  if(res==1){
    #ifdef testmodeS2
      Serial.print(" = ");
      Serial.println(EnergyKWh,3);
    #endif //testmodeS
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_EnergyKWh, EnergyKWh + _corr_EnergyKWh);
    if((millis()-pred_EnergyKWhmillis)>=3600000L || pred_EnergyKWhmillis==0){
      pred_EnergyKWhmillis = millis();
      if(_pred_EnergyKWh!=0){
        addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_EnergyKWhDelta, EnergyKWh - _pred_EnergyKWh);
      }
      _pred_EnergyKWh = EnergyKWh;
    }
  }else{
    Serial.println();
    return;
  }

  /*
  res = ElMeter_GetEnergyA(&EnergyKWh1,1);
  #ifdef testmodeS2
    Serial.print("Wh1: ");
    Serial.print(res,3);
  #endif //testmodeS
  if(res==1){
    #ifdef testmodeS2
      Serial.print(" = ");
      Serial.println(EnergyKWh1,3);
    #endif //testmodeS
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_EnergyKWh1, EnergyKWh1 + _corr_EnergyKWh1);
  }else{
    Serial.println();
    return;
  }
  res = ElMeter_GetEnergyA(&EnergyKWh2,2);
  #ifdef testmodeS2
    Serial.print("Wh2: ");
    Serial.print(res,3);
  #endif //testmodeS
  if(res==1){
    #ifdef testmodeS2
      Serial.print(" = ");
      Serial.println(EnergyKWh2,3);
    #endif //testmodeS
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_EnergyKWh2, EnergyKWh2 + _corr_EnergyKWh2);
  }else{
    Serial.println();
    return;
  } */
  
  res = ElMeter_GetInstantPower(&P1,&P2,&P3);
  #ifdef testmodeS2
    Serial.print("Power: ");
    Serial.print(res);
    Serial.print("; 1= ");
    Serial.print(P1,3);
    Serial.print(" 2= ");
    Serial.print(P2,3);
    Serial.print(" 3= ");
    Serial.println(P3,3);
  #endif //testmodeS
  if(res==1){
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_P1, P1);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_P2, P2);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_P3, P3);
  }else{
    return;
  }
  
  res = ElMeter_GetInstantVoltage(&V1,&V2,&V3);
  #ifdef testmodeS2
    Serial.print("Voltage: ");
    Serial.print(res);
    Serial.print("; 1= ");
    Serial.print(V1);
    Serial.print(" 2= ");
    Serial.print(V2);
    Serial.print(" 3= ");
    Serial.println(V3);
  #endif //testmodeS
  if(res==1){
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_V1, V1);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_V2, V2);
    addCANMessage2Queue( CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_ElMeter_V3, V3);
  }else{
    return;
  }
  
  //if(millis()-millisLastReport > 10000L){
  //  millisLastReport = millis();
    //fround( , 1)
  //}
}



