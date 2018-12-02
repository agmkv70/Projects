//NIK_defs.h
//defs of BLYNK VirtualPINs and states:

#define Status_Standby	1
#define Status_Auto1	2 //full auto (tempTargetFloorOut)
#define Status_Auto2	3 //semi-auto (ManualFloorIn)
#define Status_Manual	4 //manual valve
#define Status_Warning	5
#define Status_Error	6
int boardSTATUS = Status_Auto1;
#define VPIN_STATUS			0
#define VPIN_ErrCode		1

#define VPIN_Boiler				2
#define VPIN_FloorIn			3
#define VPIN_ManualFloorIn		4 	//semi-auto
#define VPIN_FloorOut			5
#define VPIN_Ambient			26
#define VPIN_tempTargetFloorOut	6	//full auto
#define VPIN_ManualMotorValveMinus	7 	//manual -
#define VPIN_ManualMotorValvePlus	8 	//manual +
#define VPIN_MotorValveMinus	9 		//unit's decision to move - (in automatic mode it's made automatically)
#define VPIN_MotorValvePlus		10 		//unit's decision to move + (in automatic mode it's made automatically)
#define VPIN_MainCycleInterval  11
#define VPIN_SetMainCycleInterval  12

#define VPIN_OutdoorTemp  13 //other module on CAN

#define VPIN_SetBoilerPowerPeriodMinutes  14
#define VPIN_BoilerPowerOnOff 15
#define VPIN_BoilerPower 16
#define VPIN_BoilerPID_Kp 17 
#define VPIN_BoilerPID_Ki 18 
#define VPIN_BoilerPID_Kd 19 
#define VPIN_BoilerPID_P 20 //proportional
#define VPIN_BoilerPID_I 21 //integral
#define VPIN_BoilerPID_D 22 //differential
#define VPIN_BoilerTargetTemp 23
#define VPIN_SetBoilerPID_Isum_Zero 24
#define VPIN_BoilerTargetTempGraph 25
//26 is used

#define VPIN_LEDPower12Voltage  40
#define VPIN_LEDMainCycleInterval  41
#define VPIN_LEDSetMainCycleInterval  42
#define VPIN_LEDSetPWMch1  43
#define VPIN_LEDSetPWMch2  44
#define VPIN_LEDSetPWMch3  45
#define VPIN_LEDSetPWMch4  46
