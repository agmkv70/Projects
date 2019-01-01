/* agmkv70 fork of
  emonLibCM.h - Library for openenergymonitor
  GNU GPL
  - cleared temperature and pulse count, adding 3 phase voltage mesurement
*/

// This library provides continuous single-phase monitoring of real power on up to five CT channels.
// All of the time-critical code is now contained within the ISR, only the slower activities
// are done within the main code. These slower activities include RF transmissions,
// and all Serial statements (not part of the library).  
//
// This library is suitable for either 50 or 60 Hz operation.
//
// Original Author: Robin Emley (calypso_rae on Open Energy Monitor Forum)
// Addition of Wh totals by: Trystan Lea
// Heavily modified to improve performance and calibration; by Robert Wall 
//  Release for testing 4/1/2017
//
// Version 2.0  21/11/2018
// Version 2.01  3/12/2018  No change.


#ifndef EmonLibCM_h
#define EmonLibCM_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

#define MICROSPERSEC 1.0e6

void EmonLibCM_cycles_per_second(int _cycles_per_second);
void EmonLibCM_min_startup_cycles(int _min_startup_cycles);
void EmonLibCM_datalog_period(float _datalog_period_in_seconds);
void EmonLibCM_setADC(int _ADCBits,  int ADCDuration);
void EmonLibCM_ADCCal(double _RefVoltage);
void EmonLibCM_SetADC_VChannel(byte ADC_Input, double _amplitudeCal);
void EmonLibCM_SetADC_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal);

bool EmonLibCM_acPresent(void);
int EmonLibCM_getRealPower(int channel);
int EmonLibCM_getApparentPower(int channel);
double EmonLibCM_getPF(int channel);
double EmonLibCM_getIrms(int channel);
double EmonLibCM_getVrms(void);
long EmonLibCM_getWattHour(int channel);

#ifdef INTEGRITY
int EmonLibCM_minSampleSetsDuringThisMainsCycle(void);
#endif


void EmonLibCM_Init();

void EmonLibCM_Start();

bool EmonLibCM_Ready();

// for general interaction between the main code and the ISR
extern volatile boolean datalogEventPending;


#endif
