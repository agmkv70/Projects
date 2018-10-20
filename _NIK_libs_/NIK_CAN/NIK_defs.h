//NIK_defs.h
//defs of BLYNK VirtualPINs and states:

#define Status_Standby	1
#define Status_Auto1	2 //full auto (tempTargetFloorOut)
#define Status_Auto2	3 //semi-auto (ManualFloorIn)
#define Status_Manual	4 //manual valve
#define Status_Warning	5
#define Status_Error	6
int STATUS = Status_Auto1;
#define VPIN_STATUS			0
#define VPIN_ErrCode		1

#define VPIN_Boiler				2
#define VPIN_FloorIn			3
#define VPIN_ManualFloorIn		4 	//semi-auto
#define VPIN_FloorOut			5
#define VPIN_tempTargetFloorOut	6	//full auto
#define VPIN_ManualMotorValveMinus	7 	//manual -
#define VPIN_ManualMotorValvePlus	8 	//manual +
#define VPIN_MotorValveMinus	9 		//unit's decision to move - (in automatic mode it's made automatically)
#define VPIN_MotorValvePlus		10 		//unit's decision to move + (in automatic mode it's made automatically)
#define VPIN_MainCycleInterval  11
#define VPIN_SetMainCycleInterval  12
#define VPIN_OutdoorTemp  13  //Outdoor temperature sensor