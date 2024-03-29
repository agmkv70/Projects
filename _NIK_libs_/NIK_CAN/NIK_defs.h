//NIK_defs.h
#define ErrCode_OK 0
#define ErrCode_PwrLIMIT 10

//defs of BLYNK VirtualPINs and states:

#define Status_Standby	1
#define Status_Auto1	2 //auto tFloorIn (pwm Boiler PID regulation to achieve manually set floorIn temperature)
#define Status_Auto2	3 //auto tHome (PID regulation of tFloorIn to achieve manually set Home temperature (BoilerPID works and does the actual power regulation) )
#define Status_Manual	4 //manual valve
#define Status_Warning	5
#define Status_Error	6
int boardSTATUS = Status_Auto1;
#define VPIN_STATUS			0
#define VPIN_ErrCode		1
#define VPIN_PIDSTATUS		36 //0 1 2 3 manual BoilerPower / 1 auto BoilerTemp DAY/ 2 auto HomeTemp / 3 auto BoilerTemp NIGHT
#define VPIN_VALVESTATUS	37 //0 1    manual / auto floorIn

#define VPIN_Boiler				2
#define VPIN_FloorIn			3
#define VPIN_ManualFloorIn		4 	//semi-auto
#define VPIN_FloorOut			5
#define VPIN_Home			26
#define VPIN_tempTargetFloorOut	6	//full auto
#define VPIN_ManualMotorValveMinus	7 	//manual -
#define VPIN_ManualMotorValvePlus	8 	//manual +
#define VPIN_MotorValveMinus	9 		//unit's decision to move - (in automatic mode it's made automatically)
#define VPIN_MotorValvePlus		10 		//unit's decision to move + (in automatic mode it's made automatically)
#define VPIN_MainCycleInterval  11
#define VPIN_SetMainCycleInterval  12

#define VPIN_OutdoorTemp  13 //other module on CAN

#define VPIN_SetBoilerPowerPeriodMinutes  14 //evaluation period for FloorInPID(auto1) and HomePID(auto2)
#define VPIN_BoilerPowerOnOff 15
#define VPIN_BoilerPower 16
#define VPIN_BoilerPID_Kp 17 
#define VPIN_BoilerPID_Ki 18 
#define VPIN_BoilerPID_Kd 19 
#define VPIN_BoilerPID_P 20 //proportional
#define VPIN_BoilerPID_I 21 //integral
#define VPIN_BoilerPID_D 22 //differential
#define VPIN_BoilerTargetTempDAY 23
#define VPIN_BoilerTargetTempNIGHT 38
#define VPIN_SetBoilerPID_Isum_Zero 24
#define VPIN_BoilerTargetTempGraph 25
//26 is used as VPIN_Home
#define VPIN_HomePID_Kp 27 
#define VPIN_HomePID_Ki 28 
#define VPIN_HomePID_Kd 29 
#define VPIN_HomePID_P 30 //proportional
#define VPIN_HomePID_I 31 //integral
#define VPIN_HomePID_D 32 //differential
#define VPIN_HomeTargetTemp 33
#define VPIN_SetHomePID_Isum_Zero 34
#define VPIN_HomeTargetTempGraph 35
//36 used as VPIN_PIDSTATUS
//37 used as VPIN_VALVESTATUS
//38 used as VPIN_BoilerTargetTempNIGHT

#define VPIN_LEDPower12Voltage  40
#define VPIN_LEDMainCycleInterval  41
#define VPIN_LEDSetMainCycleInterval  42
#define VPIN_LEDSetPWMch1  43
#define VPIN_LEDSetPWMch2  44
#define VPIN_LEDSetPWMch3  45
#define VPIN_LEDSetPWMch4  46

//AirHEATER:
#define VPIN_TEHPID_Kp 50 
#define VPIN_TEHPID_Ki 51 
#define VPIN_TEHPID_Kd 52 
#define VPIN_TEHPID_P 53 //proportional
#define VPIN_TEHPID_I 54 //integral
#define VPIN_TEHPID_D 55 //differential
#define VPIN_SetTEHPID_Isum_Zero 56
#define VPIN_AirOutTargetTemp    57
#define VPIN_AirOutTargetTempGraph    58
#define VPIN_TEHPower                 59
#define VPIN_SetTEHPowerPeriodSeconds 60

#define VPIN_AirInTemp     61
#define VPIN_HumidityAirIn 62
#define VPIN_TEHTemp       63
#define VPIN_AirOutTemp    64
#define VPIN_TEHMaxTemp    65
#define VPIN_ClearTEHOverheatError  67
//75 ?
#define VPIN_HEATER_TEHPIDSTATUS    76     //0 1 2 (off/on/error)
#define VPIN_HEATER_VALVESTATUS     77     //0 1 (closed/opened), 2(opening), 3(closing)
#define VPIN_HEATER_SetReadTempCycleInterval 78
#define VPIN_HEATER_TEHERROR        79
//#define VPIN_HEATER_ERR_TEHOVERHEAT    80
#define VPIN_TEHTargetTemp          81
#define VPIN_TEHPower_PIDCorrection     82 //был граф TEHTargetTemp, но не использовался
#define VPIN_TEH_KdTempAirIn         83 //coef
#define VPIN_HEATER_PIN4READ        84

#define VPIN_TEH_kPwr               85 //=kPwr2Air, for TRGSTATUS=4 //fan+heat(kPwr)
#define VPIN_TEH_kPwr_preMillisPerC 86 //starting millis for full power preheat millis/*C
#define VPIN_TEH_ErrType 87
#define VPIN_TEH_kPwrPreheatON 88

#define VPIN_HEATER_FREERAM          89 //send avr free ram to blynk
#define VPIN_BLYNK_TERMINAL          90 //gets strings up to 8 bytes
#define VPIN_HEATER_TRGSTATUS_OpCl   91 //0=open /1=close
#define VPIN_HEATER_TRGSTATUS_HEAT   92 //0=off  /1=heat
#define VPIN_HEATER_TRGSTATUS        93     //off+close=0,1//off+open=2//fan+open=3//fan+heat(kPwr)=4//fan+heat(PID)=5

#define VPIN_OutdoorAirQ_Temperature  94
#define VPIN_OutdoorAirQ_Humidity     95
#define VPIN_OutdoorAirQ_Pressure     96
#define VPIN_OutdoorAirQ_Gas          97
//#define VPIN_OutdoorAirQ_Altitude
#define VPIN_OutdoorAirQ_IAQ          98
#define VPIN_OutdoorAirQ_IAQ_accuracy 99 //0-starting; 1-too stable env; 2-(re)calibrating; 3-working

#define VPIN_CANQueueError 100 //CANQueueError on Wifi-ESP8266

//////ElMeter
#define VPIN_ElMeter_EnergyKWh     110
#define VPIN_ElMeter_EnergyKWhDelta     111
#define VPIN_ElMeter_P1     112
#define VPIN_ElMeter_P2     113
#define VPIN_ElMeter_P3     114
#define VPIN_ElMeter_V1     115
#define VPIN_ElMeter_V2     116
#define VPIN_ElMeter_V3     117
#define VPIN_ElMeter_EnergyKWh1     118
#define VPIN_ElMeter_EnergyKWh2     119
#define VPIN_ElMeter_EnergyKWh_cor  120
#define VPIN_ElMeter_EnergyKWh1_cor 121
#define VPIN_ElMeter_EnergyKWh2_cor 122

#define VPIN_Floor_ECHO     123

#define VPIN_BAT_VoltmeterV   124
#define VPIN_BAT_VoltmeterA   125
#define VPIN_BAT_EnergyWH     126
#define VPIN_BAT_MaxEnergyWH  127
#define VPIN_BAT_EnergyPercent   128
#define VPIN_BAT_CurPowerW       129
#define VPIN_BAT_SendIntervalSec 130
