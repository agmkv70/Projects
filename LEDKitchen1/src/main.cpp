//#define testmode

#define CAN_PIN_INT 8
#define CAN_PIN_CS  7 

#include <NIK_defs.h>
#include <NIK_can.h>

#define PWMpin1 9  //3 //31k
#define PWMpin2 10 //5 //62k
#define PWMpin3 3  //6 //62k
#define PWMpin4 5  //9 //31k
#define PWMpin1div 8
#define PWMpin2div 8
#define PWMpin3div 8
#define PWMpin4div 8

#define AVreadpin 0 //A0

int mainTimerId,measure12vId;
float average12v;
int eepromVIAddr=1000,eepromValueIs=7650+0; //if this is in eeprom, then we got valid values, not junk

int MainCycleInterval=300, PWMch1=0, PWMch2=0, PWMch3=0, PWMch4=0;

//float fround(float r, byte dec){
//	if(dec>0) for(byte i=0;i<dec;i++) r*=10;
//	r=(long)(r+0.5);
//	if(dec>0) for(byte i=0;i<dec;i++) r/=10;
//	return r;
//}

void Cycle_measure12v(){
  float avolt = ((float)analogRead(AVreadpin)/1000*15.492773); //reading of 12V
  float K=0.01;
  average12v = (1-K)*average12v + K*avolt; //K=[0..1]      t = (1-K) * dt / K      dt=0.050s; K=0.01; t=4.95s(supress waves)
}

void MainCycle_StartEvent(){
  //digitalWrite(13,HIGH);

  float avolt = average12v;//((float)analogRead(AVreadpin)/1000*15.492773); //reading of 12V
    
  #ifdef testmode
		Serial.print("a0volt = ");
		Serial.println(avolt);
  #endif

  addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_LEDPower12Voltage,fround(avolt,2)); //rounded 0.0 value
	//digitalWrite(13,LOW);
}

char setReceivedVirtualPinValue(unsigned char vPinNumber, float vPinValueFloat){
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
		case VPIN_LEDSetPWMch1:  PWMch1  = vPinValueFloat; analogWrite(PWMpin1,PWMch1); EEPROM_storeValues(); break;
		case VPIN_LEDSetPWMch2:  PWMch2  = vPinValueFloat; analogWrite(PWMpin2,PWMch2); EEPROM_storeValues(); break;
		case VPIN_LEDSetPWMch3:  PWMch3  = vPinValueFloat; analogWrite(PWMpin3,PWMch3); EEPROM_storeValues(); break;
		case VPIN_LEDSetPWMch4:  PWMch4  = vPinValueFloat; analogWrite(PWMpin4,PWMch4); EEPROM_storeValues(); break;
		default:
			return 0;
	}
	return 1;
}

void EEPROM_storeValues(){
  EEPROM_WriteAnything(eepromVIAddr,eepromValueIs);
  
  EEPROM.update(VPIN_STATUS,(unsigned char)boardSTATUS);
  EEPROM.update(VPIN_LEDMainCycleInterval,(unsigned char)(MainCycleInterval/10));

  EEPROM.update(VPIN_LEDSetPWMch1,(unsigned char)PWMch1);
  EEPROM.update(VPIN_LEDSetPWMch2,(unsigned char)PWMch2);
  EEPROM.update(VPIN_LEDSetPWMch3,(unsigned char)PWMch3);
  EEPROM.update(VPIN_LEDSetPWMch4,(unsigned char)PWMch4);

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
  
  PWMch1 = EEPROM.read(VPIN_LEDSetPWMch1);
  PWMch2 = EEPROM.read(VPIN_LEDSetPWMch2);
  PWMch3 = EEPROM.read(VPIN_LEDSetPWMch3);
  PWMch4 = EEPROM.read(VPIN_LEDSetPWMch4);
}

/** Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0 (delay,millis)
 *   - Pins 9 and 10 are paired on timer1 (servo)
 *   - Pins 3 and 11 are paired on timer2 (tone)
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}

void setup() {

  EEPROM_restoreValues();
  //timer.setInterval(1000L*600L, EEPROM_storeValues); //once in 10 min remember critical values
  average12v=0;

  #ifdef testmode
  Serial.begin(115200);
  Serial.println("Start");
  #endif

  pinMode(PWMpin1,OUTPUT);
  pinMode(PWMpin2,OUTPUT);
  pinMode(PWMpin3,OUTPUT);
  pinMode(PWMpin4,OUTPUT);
  //digitalWrite(PWMpin1,0);

  setPwmFrequency(PWMpin1,PWMpin1div);
  setPwmFrequency(PWMpin2,PWMpin2div);
  setPwmFrequency(PWMpin3,PWMpin3div);
  //setPwmFrequency(PWMpin4,PWMpin4div);

  analogWrite(PWMpin1,PWMch1);
  analogWrite(PWMpin2,PWMch2);
  analogWrite(PWMpin3,PWMch3);
  analogWrite(PWMpin4,PWMch4);

  // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
  //if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
    ;//Serial.println("CAN bus OK: MCP2515 Initialized Successfully!");
  else
  {  
		#ifdef testmode
		Serial.println("Error Initializing CAN bus driver MCP2515...");
		#endif
	}

  //receive 0x100 messages:
  CAN0.init_Mask(0,0,0x01FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x01000000);                // Init first filter...
   
  #ifdef testmode
	CAN0.setMode(MCP_LOOPBACK);
	#endif
	#ifndef testmode
  CAN0.setMode(MCP_NORMAL);  // operation mode to normal so the MCP2515 sends acks to received data.
	#endif
  pinMode(CAN_PIN_INT, INPUT);  // Configuring CAN0_INT pin for input

  //pinMode(13,OUTPUT);//led

  mainTimerId = timer.setInterval(1000L*MainCycleInterval, MainCycle_StartEvent); //start regularly
  measure12vId = timer.setInterval(50L,Cycle_measure12v);
}

// int i=0;
// int way=1;
void loop() {
    //system events:
    timer.run();
    checkReadCAN();

    ////////other code:
    //analogWrite(PWMpin1,255);
    //delay(20);

    //analogWrite(PWMpin1,i);
    //analogWrite(PWMpin2,i);
    //analogWrite(PWMpin3,i);
    //analogWrite(PWMpin4,i);
    //delay(20);
    
    //i=i+way;
    //if(i>55)
    //{ way=-1; i=i+way; }
    //if(i<0)
    //{ way=1; i=i+way; }
}

