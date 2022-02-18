#include <Arduino.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

//SimpleTimer timer;

//#define HardSerial
//#define testMode

//-------- порты для CAN
#ifndef HardSerial
#define SSerialTx 3     // was 1
#define SSerialRx 2     // was 0
SoftwareSerial CANSerial(SSerialRx, SSerialTx); // Rx, Tx
#endif

//    команды Меркурий 230:
//    |Адрес счетчика 1 байт | Запрос 1 байт|
byte address[] = {232};// адрес мой 232

byte test_cmd[] = {0}; // тестирование канала связи
byte openUser_cmd1[] = {1,1,1,1,1,1,1,1}; // тестирование канала связи (code=1,level=1,pw=111111(hex))
byte openUser_cmd2[] = {1,2,2,2,2,2,2,2}; // тестирование канала связи (code=1,level=2,pw=222222(hex))

byte getTime_cmd[] = {4,0};    // read=4, cur.time=0
byte setTime_cmd[] = {3,0x0C, 0,0,0,0,0,0,0,0}; // ss,mm,hh,wd,MM,YY,(s=0/w=1)
byte setTimeCorr_cmd[] = {3,0x0D, 0,0,0};       // ss,mm,hh +-4min/day

byte getEnergy_cmd[] = {5,0x00,0};    // readEnergy=5, from=0+month=0, tarif=0(sum=0)
byte curent_PSum_cmd[] = {8,0x16,0x00}; //phase sum=0x00, 0x01-1phase,...
//byte curent_P1_cmd[] = {8,0x14,0x01}; //Power 1 phase
//byte curent_P2_cmd[] = {8,0x14,0x02};
//byte curent_P3_cmd[] = {8,0x14,0x03};
/*byte curent_I1_cmd[] = {8,0x14,0x21}; //Current 1 phase
byte curent_I2_cmd[] = {8,0x14,0x22};
byte curent_I3_cmd[] = {8,0x14,0x23};*/
byte curent_Uall_cmd[] = {8,0x16,0x11}; //U all phase?
//byte curent_U1_cmd[] = {8,0x14,0x11}; //U 1 phase
//byte curent_U2_cmd[] = {8,0x14,0x12};
//byte curent_U3_cmd[] = {8,0x14,0x13};

#define MAXRESPONSE 32
byte response[MAXRESPONSE]; // длина массива входящего сообщения
int byteReceived;
int byteSend;

unsigned int crc16MODBUS(byte *s, int count){ // Расчет контрольной суммы для запроса
  unsigned int crcTable[] = {
      0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
      0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
      0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
      0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
      0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
      0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
      0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
      0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
      0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
      0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
      0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
      0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
      0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
      0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
      0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
      0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
      0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
      0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
      0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
      0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
      0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
      0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
      0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
      0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
      0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
      0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
      0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
      0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
      0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
      0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
      0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
      0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

  unsigned int crc = 0xFFFF;

  for (int i = 0; i < count; i++){
    crc = ((crc >> 8) ^ crcTable[(crc ^ s[i]) & 0xFF]);
  }
  return crc;
}

#ifdef HardSerial
void SerialCleanSwap(){
  //clear read buffer
  while(Serial.available()){
    Serial.read();
  }
  Serial.flush(); //write everything
  Serial.swap();
}
#endif

byte test_send(byte *cmd, int s_cmd, byte responseLength){ // для тестовых запросов {Адрес счетчика 4, 	Запрос 1, 	CRC16 (Modbus) 2}
  int s_address = sizeof(address);
  int s_address_cmd = s_address + s_cmd;
  int s_address_cmd_crc = s_address_cmd + 2;

  byte address_cmd_crc[s_address_cmd_crc];

  //write adress:
  int pos = 0;
  for (int i = 0; i < s_address; i++){
    address_cmd_crc[pos++] = address[i];
  }
  //add command:
  for (int i = 0; i < s_cmd; i++){
    address_cmd_crc[pos++] = cmd[i];
  }

  unsigned int crc = crc16MODBUS(address_cmd_crc, s_address_cmd);
  byte crc1 = crc & 0xFF;
  byte crc2 = (crc >> 8) & 0xFF;
  address_cmd_crc[pos++] = crc1;
  address_cmd_crc[pos] = crc2;

  #ifdef testMode
    //print command:
    String temp_term2 = "";
    for (int i = 0; i < s_address_cmd_crc; i++){
      temp_term2 += String(address_cmd_crc[i], HEX);
      temp_term2 += " ";
    }
    Serial.print("Send HEX:  ");
    Serial.println(temp_term2);
  #endif
  
  #ifdef HardSerial
    SerialCleanSwap();
  #else
    CANSerial.flush();
    while(CANSerial.available()){
      CANSerial.read();
    }
  #endif

  //send:
  for (int i = 0; i < s_address_cmd_crc; i++){
    #ifdef HardSerial
      Serial.write(address_cmd_crc[i]);
    #else
      CANSerial.write(address_cmd_crc[i]);
    #endif
  }
  #ifdef HardSerial
    Serial.flush();
  #else
    CANSerial.flush();
    while(CANSerial.available()){
      CANSerial.read();
    }
  #endif
  //delay(10);
  
  //receive:
  unsigned long receivetimeout=150, startmillis=millis(), lastreadmillis;
  lastreadmillis = startmillis;
  byte irec = 0;
  do{ 
    delay(5);
    #ifdef HardSerial
      while(Serial.available()){
        byteReceived = Serial.read(); 
        response[irec++] = byteReceived;
        lastreadmillis = millis();
      }
    #else
      while(CANSerial.available()){
        byteReceived = CANSerial.read();
        response[irec++] = byteReceived;
        lastreadmillis = millis();
      }
    #endif
  }while(irec < //s_address_cmd_crc+
                responseLength && millis()-lastreadmillis < receivetimeout && irec<MAXRESPONSE);
  
  #ifdef HardSerial
    SerialCleanSwap();
  #else
  #endif

  //print received string:
  if(irec>0){
    #ifdef testMode
      String temp_term1 = "";
      for (unsigned int i = 0; i < irec; i++){
        temp_term1 += String(response[i], HEX);
        temp_term1 += " ";
      }
      Serial.print("Received:  ");
      Serial.print(temp_term1);
      Serial.print(" - ");
      Serial.print(millis()-startmillis);
      Serial.print(" : ");
      Serial.print(irec);
      Serial.print(" < ");
      Serial.print(//s_address_cmd_crc+
                    responseLength);
      Serial.println("");
    #endif
    if(irec==responseLength && irec>2){
      //check CRC
      crc = crc16MODBUS(response, irec-2);
      crc1 = crc & 0xFF;
      crc2 = (crc >> 8) & 0xFF;
      if(response[irec-2] == crc1 && response[irec-1] == crc2){
        return 1;//OK
      }else{
        return -2;//crc error
      }
    }else{
      return -1;//incorrect length
    }
  }else{
    return 0;//no answear
  }
}

byte ElMeter_TestConnection(){
  byte res = test_send(test_cmd, sizeof(test_cmd), 4);
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
    res = test_send(openUser_cmd1, sizeof(openUser_cmd1), 4);
  }else if(usr==2){
    res = test_send(openUser_cmd2, sizeof(openUser_cmd2), 4);
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
  
byte ElMeter_GetTime(byte *YY,byte *MM,byte *DD,byte *hh,byte *mm,byte *ss,byte *sumerWinter){
  byte res;
  res = test_send(getTime_cmd, sizeof(getTime_cmd), 11);
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
  res = test_send(setTime_cmd, sizeof(setTime_cmd), 4);
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
  res = test_send(setTimeCorr_cmd, sizeof(setTimeCorr_cmd), 4);
    
  if(res>0){
    if(response[0]==address[0]){
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_GetEnergyA(float *Active,byte tariff){
  byte res;
  getEnergy_cmd[2]=tariff;
  res = test_send(getEnergy_cmd, sizeof(getEnergy_cmd), 19); 
  //4x4 bytes (A+,A-,R+,R-)
  
  if(res>0){
    if(response[0]==address[0]){
      
      byte b1 = response[1];
      byte b2 = response[2];
      byte b3 = response[3];
      byte b4 = response[4];
      //b2_b1_b4_b3     68 1 59 10 > 01681059
      unsigned long _Active = (unsigned long)b2<<24 | (unsigned long)b1<<16 | (unsigned long)b4<<8 | (unsigned long)b3; 
      *Active = (float)_Active/1000;
      
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_GetInstantPower(float *Ph1,float *Ph2,float *Ph3){
  byte res;
  res = test_send(curent_PSum_cmd, sizeof(curent_PSum_cmd), 15); 
  //4x3 bytes (sum,ph1,ph2,ph3)
  
  if(res>0){
    if(response[0]==address[0]){
      
      byte b1 = response[4];
      byte b2 = response[5];
      byte b3 = response[6];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      unsigned long P = (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph1 = (float)P/1000;
      
      b1 = response[7];
      b2 = response[8];
      b3 = response[9];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      P = (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph2 = (float)P/1000;
      
      b1 = response[10];
      b2 = response[11];
      b3 = response[12];
      //b1_b3_b2 (b1 = 1bit_ActSign,2bit_ReaSign,345678)
      P = (unsigned long)b3<<8 | (unsigned long)b2;
      *Ph3 = (float)P/1000;
      
      return 1; //OK
    }else{
      return -10; //wrong answear
    }
  }else
    return res;
}

byte ElMeter_GetInstantVoltage(float *Ph1,float *Ph2,float *Ph3){
  byte res;
  res = test_send(curent_Uall_cmd, sizeof(curent_Uall_cmd), 12); 
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

void setup(){ 
  //Serial.begin(9600);
  Serial.begin(115200);
  #ifdef HardSerial
  #else
    CANSerial.begin(9600);
    if (!CANSerial) { // If the object did not initialize, then its configuration is invalid
      Serial.println("!!Invalid SoftwareSerial pin configuration, check config!!"); 
      int on=1;
      while (1) { // Don't continue with invalid configuration
        digitalWrite(LED_BUILTIN,on);
        delay (300);
        on^=1;
      }
    } 
  #endif
  
  Serial.println("***setup OK***");
  
  //timer.setInterval(1000L, Mercury_Fast_Data); // интервал для частых  опросов (1000L = 1 секунда)
  //pinMode(SerialControl, OUTPUT);
  /*pinMode(0, INPUT);
  pinMode(1, INPUT);*/

  //byte res = ElMeter_SetTimeCorr(20,05,00);
  //Serial.print("SetTime: ");
  //Serial.println(res);

}

void loop(){ 
  Serial.println();

  byte res = ElMeter_TestConnection();
  Serial.print("Test connection: ");
  Serial.println(res);
  
  res = ElMeter_OpenUser(1);
  Serial.print("Login user1: ");
  Serial.println(res);

  byte YY,MM,DD,hh,mm,ss,sumerWinter;
  res = ElMeter_GetTime(&YY,&MM,&DD,&hh,&mm,&ss,&sumerWinter);
  Serial.print("Get time: ");
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
  res = ElMeter_GetEnergyA(&Wh,0);
  Serial.print("Wh: ");
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
  res = ElMeter_GetInstantPower(&ph1,&ph2,&ph3);
  Serial.print("Power: ");
  Serial.print(res);
  Serial.print("; 1= ");
  Serial.print(ph1,3);
  Serial.print(" 2= ");
  Serial.print(ph2,3);
  Serial.print(" 3= ");
  Serial.println(ph3,3);
  
  res = ElMeter_GetInstantVoltage(&ph1,&ph2,&ph3);
  Serial.print("Voltage: ");
  Serial.print(res);
  Serial.print("; 1= ");
  Serial.print(ph1);
  Serial.print(" 2= ");
  Serial.print(ph2);
  Serial.print(" 3= ");
  Serial.println(ph3);

  /*
  test_send(test_cmd, sizeof(test_cmd), 4);
  test_send(openUser_cmd1, sizeof(openUser_cmd1), 4);
  test_send(getTime_cmd, sizeof(getTime_cmd), 11); //(adr) 03 28 21,01,31,01 22,01 (CRC): 21:28:03,Monday,31,jan,2022y,winter(=1)
  //Ответ: (80) 43 14 16 03 27 02 08 01 (CRC).
  //Результат: 16:14:43 среда 27 февраля 2008 года, зима
  test_send(getEnergy_cmd, sizeof(getEnergy_cmd), 19); //4x4 bytes (A+,A-,R+,R-)
  test_send(curent_PSum_cmd, sizeof(curent_PSum_cmd), 15); //4x3 bytes (sum,ph1,ph2,ph3)
  test_send(curent_Uall_cmd, sizeof(curent_Uall_cmd), 12); //3x3 (ph1,ph2,ph3)
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
  
  delay(8000);

  //timer.run();
}