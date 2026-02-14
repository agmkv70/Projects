/*
 *  JK-BMSToPylontechCAN.cpp
 *
 *  Converts the JK-BMS RS485 data to Pylontech CAN data for inverters
 *  which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.
 *  It displays many BMS information and alarms on a locally attached 2004 LCD.
 *  The JK-BMS data are provided as RS232 TTL.
 *
 *  Data from JK-BMS is received by the Hardware USART, and sent by the SoftwareSerialTX library.
 *  Data for serial monitor is sent by Hardware USART (connected to USB).
 *
 *  The software TX and hardware RX lines are connected to the JK-BMS and run with 115200 baud.
 *  CAN is connected to the inverter which must accept Pylontech low voltage protocol, which runs with 500 kBit/s.
 *  This protocol is used e.g by the Pylontech US2000 battery.
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!! 8 MHz crystal and 500 kBit/s does not work with MCP2515                 !!!
 *  !!! So you must replace the crystal of the module with a 16 (or 20) MHz one !!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *  Internal operation (default every 2 seconds):
 *  1. A request to deliver all informations is sent to the BMS (1.85 ms).
 *  2. Wait and receive the BMS status frame (0.18 to 1 ms + 25.5 ms).
 *  3. The BMS status frame is stored in a buffer and parity and other plausi checks are made.
 *  4. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
 *     Other frame data are mapped to a C structure.
 *     But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
 *  5. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
 *  6. The content of the status frame is printed. After reset, all info is printed once, then only dynamic info is printed.
 *  7. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
 *  8. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
 *  9. CAN data is sent..
 *
 *  The LCD has 4 "pages" showing overview data, up to 16 cell voltages, up to 16 cell minimum and maximum statistics, or SOC and current with big numbers.
 *  The pages can be switched by the button at pin 2.
 *
 *  On timeout, the last BMS data is kept.
 *
 *  It uses the following libraries, which are included in this repository:
 *  SoftwareSerialTX for sending Serial to JK-BMS
 *  Modified LiquidCrystal_I2C for I2C connected LCD
 *  LCDBigNumbers for LCD big number generation
 *  EasyButtonAtInt01 for LCD page switching button
 *  SoftI2CMaster for minimal I2C functions
 *  modified mcp_can_dfs.h file from Seed-Studio Seeed_Arduino_CAN
 *
 *  Based on https://github.com/syssi/esphome-jk-bms and https://github.com/maxx-ukoo/jk-bms2pylontech
 *  Tested with SUN-5K-SG03LP1-EU
 *
 *  Available as Wokwi example https://wokwi.com/projects/371657348012321793
 *
 *  Copyright (C) 2023-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/JK-BMSToPylontechCAN.
 *
 *  JK-BMSToPylontechCAN is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 *
 * # Connection schematic
 * A schottky diode is inserted into the RX line to allow programming the AVR with the JK-BMS still connected.
 *
 * ALTERNATIVE EXTERNAL POWER SUPPLY:
 *                                          78L05    Optional Schottky diode - From Uno/Nano 5 V
 *                                           ___                         to enable powering CAN
 * Optional 6.6 V from Battery #2 >-o-------|___|-------o-|<|-< Uno 5V   module by Nano USB, if
 *                                  |         |         |                battery is not attached
 *                                  |        ---        |
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 *                                  |                   |
 *  __________ Schottky diode   ____|____           ____|____             _________
 * |        TX|>---|<|-- RX -->|RX Vin   |<- SPI ->|   5V    |           |         |
 * |        RX|<-------- TX --<|4  Uno/  |         | MCP2515 |           |         |
 * |  JK-BMS  |                |   Nano  |         |   CAN   |<-- CAN -->|  DEYE   |
 * |          |<------- GND -->|         |<- GND ->|         |           |         |
 * |__________|                |_________|         |_________|           |_________|
 *
 * # Connection diagram for JK-BMS GPS / UART-TTL socket (4 Pin, JST 1.25mm pitch)
 *  ___ ________ ___
 * |                |
 * | O   O   O   O  |
 * |GND  RX  TX VBAT|
 * |________________|
 *   |   |   |
 *   |   |   --|<|-- RX of Uno / Nano
 *   |   ----------- D4 (or other pin, if specified)
 *   --------------- GND
 */

#include <Arduino.h>
// For full revision history see https://github.com/ArminJo/JK-BMSToPylontechCAN?tab=readme-ov-file#revision-history
#define VERSION_EXAMPLE "3.2.1_nik"

#define CAN_PIN_INT 9
#define CAN_PIN_CS  10 

#include <NIK_defs.h>
#include <NIK_can.h>

#include <avr/io.h>
#include <avr/interrupt.h>
int mainTimerId;
int eepromVIAddr=1000,eepromValueIs=7510+0; //if this is in eeprom, then we got valid values, not junk
int MainCycleInterval=6; //seconds

#define LED_WS2812_PIN 3 
#define NUMWSLEDS 16     
#include <microLED.h>   
microLED <NUMWSLEDS, LED_WS2812_PIN, MLED_NO_CLOCK, LED_WS2812, ORDER_GRB> LED_WS2812_strip;
int LED_WS_count=0;
mData LED_WS_color;
byte LED_WS_blinkState=0;

void LED_WS2812_Update(){ //once in a second
    LED_WS2812_strip.setBrightness(30); //gluchilo - maybe helps
    LED_WS2812_strip.clear();
    if(LED_WS_count == -1){
        LED_WS2812_strip.fill(8,4,LED_WS_color); //light 2 middle leds
    }else{
        LED_WS2812_strip.fill(0,LED_WS_count-1,LED_WS_color);
    }
    //blink with 2 leds for illumination: 
    LED_WS2812_strip.set(4+LED_WS_blinkState, mWhite);
    LED_WS_blinkState = LED_WS_blinkState==0 ? 1 : 0;
    LED_WS2812_strip.show();
}

/*
 * If battery SOC is below this value, the inverter is forced to charge the battery from any available power source regardless of inverter settings.
 */
//#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        5
#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        0 // This disables the setting if the force charge request, even if battery SOC is 0.

/*
 * Macros for CAN data modifications
 */
//#define CAN_DATA_MODIFICATION         // Currently enables the function to reduce max current at high SOC level
//#define USE_CCCV_MODIFY_FUNCTION      // Changes modification to CCCV method by Ngoc: https://github.com/ArminJo/JK-BMSToPylontechCAN/discussions/31
//#define USE_OWN_MODIFY_FUNCTION       // Use (currently empty) function which must be filled in at bottom of Pylontech_CAN.hpp
/*
 * Values for standard CAN data modification
 */
//#define MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT        80  // Start SOC for linear reducing maximum current. Default 80
//#define MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE       50  // Value of current at 100 % SOC. Units are 100 mA! Default 50
//#define DEBUG                 // This enables debug output for all files - only for development
//#define STANDALONE_TEST       // If activated, fixed BMS data is sent to CAN bus and displayed on LCD.
#define USE_NO_LCD
#if defined(STANDALONE_TEST)
//#define LCD_PAGES_TEST // Additional automatic tests
#  if defined(LCD_PAGES_TEST)
//#define BIG_NUMBER_TEST // Additional automatic tests, especially for rendering of JK_BMS_PAGE_BIG_INFO
#  endif
//#define ENABLE_MONITORING
#  if defined(USE_NO_LCD)
#     undef USE_NO_LCD // LCD is activated for standalone test
#  endif
#endif

#if defined(__AVR_ATmega644P__)
#   define USE_LAYOUT_FOR_644_BOARD
#endif

/*
 * Options to reduce program size / add optional features
 */
#if FLASHEND > 0x7FFF // for more than 32k
#define ENABLE_MONITORING // Requires additional 858 bytes program space
#define SERIAL_INFO_PRINT // Requires additional 1684 bytes program space
#endif
//#define KEEP_ANALYTICS_ACCUMULATED_DATA_AT_RESET  // Requires additional 80 bytes program space

#define DO_NOT_SHOW_SHORT_CELL_VOLTAGES // Saves 470 bytes program space
#if !defined(DO_NOT_SHOW_SHORT_CELL_VOLTAGES)
#define SHOW_SHORT_CELL_VOLTAGES // Print 3 digits cell voltage (value - 3.0 V) on Cell Info page. Enables display of up to 20 voltages or additional information.
#endif

#if defined(ENABLE_MONITORING) && !defined(MONOTORING_PERIOD_SECONDS)
//#define MONOTORING_PERIOD_FAST    // If active, then print CSV line every 2 seconds, else every minute
#define MONOTORING_PERIOD_SLOW    // If active, then print CSV line every hour, and CSV line every 10 minutes
#endif

#if !defined(SERIAL_INFO_PRINT) && !defined(STANDALONE_TEST) && FLASHEND <= 0x7FFF
#define NO_SERIAL_INFO_PRINT    // Disables writing some info to serial output. Saves 974 bytes program space.
#endif

#if defined(NO_SERIAL_INFO_PRINT)
#define JK_INFO_PRINT(...)      void();
#define JK_INFO_PRINTLN(...)    void();
#else
#define JK_INFO_PRINT(...)      Serial.print(__VA_ARGS__);
#define JK_INFO_PRINTLN(...)    Serial.println(__VA_ARGS__);
#endif

#define NO_CAPACITY_35F_EXTENSIONS // Disables generating of frame 0x35F for total capacity. This additional frame is no problem for Deye inverters. Saves 56 bytes program space.
#define NO_CAPACITY_379_EXTENSIONS // Disables generating of frame 0x379 for total capacity. This additional frame is no problem for Deye inverters. Saves 24 bytes program space.
#define NO_BYD_LIMITS_373_EXTENSIONS // Disables generating of frame 0x373 for cell limits as sent by BYD battery. See https://github.com/dfch/BydCanProtocol/tree/main. This additional frame is no problem for Deye inverters. Saves 200 bytes program space.
#define NO_CELL_STATISTICS    // Disables generating and display of cell balancing statistics. Saves 1628 bytes program space.
#if !defined(STANDALONE_TEST)
    #define NO_ANALYTICS          // Disables generating, storing and display of SOC graph for Arduino Serial Plotter. Saves 3856 bytes program space.
#endif
#define USE_NO_LCD            // Disables the code for the LCD display. Saves 25% program space on a Nano.

//#define USE_NO_COMMUNICATION_STATUS_LEDS // The code for the BMS and CAN communication status LED is deactivated and the pins are not switched to output

#if !defined(ENABLE_LIFEPO4_PLAUSI_WARNING)
#define SUPPRESS_LIFEPO4_PLAUSI_WARNING     // Disables warning on Serial out about using LiFePO4 beyond 3.0 v to 3.45 V.
#endif

#if !defined(USE_NO_LCD)
#define USE_SERIAL_2004_LCD // Parallel or 1604 LCD not yet supported
#define LCD_MESSAGE_PERSIST_TIME_MILLIS 2000
#endif

#if defined(ARDUINO_AVR_NANO)
// Save space for an unmodified Nano
#  if !defined(NO_CELL_STATISTICS)
#define NO_CELL_STATISTICS          // No cell values, cell minimum, maximum and percentages.
#  endif
#endif

// sStringBuffer is defined in JK-BMS_LCD.hpp if not ENABLE_MONITORING and NO_ANALYTICS are defined
#if defined(ENABLE_MONITORING)
char sStringBuffer[100];                 // For cvs lines, "Store computed capacity" line and LCD rows
#elif !defined(NO_ANALYTICS)
char sStringBuffer[40];                 // for "Store computed capacity" line, printComputedCapacity() and LCD rows
#endif

/*
 * Pin layout, may be adapted to your requirements
 */
#define BUZZER_PIN                             A2 // To signal errors
#define PAGE_SWITCH_DEBUG_BUTTON_PIN_FOR_INFO   2 // Button at INT0 / D2 for switching LCD pages - definition is not used in program, only for documentation.
// The standard RX of the Arduino is used for the JK_BMS connection.
#define JK_BMS_RX_PIN_FOR_INFO                  0 // We use the Serial RX pin - definition is not used in program, only for documentation.
#if !defined(JK_BMS_TX_PIN)                       // Allow override by global symbol
#  if defined(USE_LAYOUT_FOR_644_BOARD)
#define JK_BMS_TX_PIN                          12
#  else
#define JK_BMS_TX_PIN                           4
#  endif
#endif

#ifdef USE_NO_COMMUNICATION_STATUS_LEDS
#   define TURN_BMS_STATUS_LED_ON                  void()
#   define TURN_BMS_STATUS_LED_OFF                 void()
#   define TURN_CAN_STATUS_LED_ON                  void()
#   define TURN_CAN_STATUS_LED_OFF                 void()
#else
// BMS and CAN communication status LEDs
#  if !defined(BMS_COMMUNICATION_STATUS_LED_PIN)
#    if defined(USE_LAYOUT_FOR_644_BOARD)
#   define BMS_COMMUNICATION_STATUS_LED_PIN        14
#   define CAN_COMMUNICATION_STATUS_LED_PIN        15
#    else
#       define BMS_COMMUNICATION_STATUS_LED_PIN        6
#       define CAN_COMMUNICATION_STATUS_LED_PIN        7
#    endif
#  endif
#   define TURN_BMS_STATUS_LED_ON                  digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, HIGH)
#   define TURN_BMS_STATUS_LED_OFF                 digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, LOW)
#   define TURN_CAN_STATUS_LED_ON                  digitalWriteFast(CAN_COMMUNICATION_STATUS_LED_PIN, HIGH)
#   define TURN_CAN_STATUS_LED_OFF                 digitalWriteFast(CAN_COMMUNICATION_STATUS_LED_PIN, LOW)
#endif

#if !defined(NO_ANALYTICS)
#define DISABLE_ESR_IN_GRAPH_OUTPUT_PIN         8 // If this pin is held low, the ESR value is not used to correct the voltage in the graph output.
#endif

/*
 * The SPI pins for connection to CAN converter and the I2C / TWI pins for the LCD are determined by hardware.
 * For Uno / Nano:
 *   SPI: MOSI - 11, MISO - 12, SCK - 13. CS cannot be replaced by constant ground.
 *   I2C: SDA - A4, SCL - A5.
 */
#if defined(USE_LAYOUT_FOR_644_BOARD)
#   define SPI_CS_PIN                              4 // !SS Must be specified before #include "MCP2515_TX.hpp"
#   define SPI_MOSI_PIN_FOR_INFO                   5 // Definition is not used in program, only for documentation.
#   define SPI_MISO_PIN_FOR_INFO                   6 // Definition is not used in program, only for documentation.
#   define SPI_SCK_PIN_FOR_INFO                    7 // Definition is not used in program, only for documentation.
#else
//#  if !defined(SPI_CS_PIN)                          // Allow override by global symbol
#define SPI_CS_PIN                              5 // second - to INVERTER!
//# define SPI_CS_PIN                             10 // primary - to NIK's CANBUS! //Must be specified before #include "MCP2515_TX.hpp"
#   define SPI_MOSI_PIN_FOR_INFO                  11 // Definition is not used in program, only for documentation.
#   define SPI_MISO_PIN_FOR_INFO                  12 // Definition is not used in program, only for documentation.
#   define SPI_SCK_PIN_FOR_INFO                   13 // Definition is not used in program, only for documentation.
//#  endif
#endif // defined(USE_LAYOUT_FOR_644_BOARD)

/*
 * Program timing, may be adapted to your requirements
 */
#if defined(STANDALONE_TEST)
#   define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     1000
#   define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             1000
#   define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "1" // Only for display on LCD
#   define SECONDS_BETWEEN_CAN_FRAME_SEND                  "1" // Only for display on LCD
#else
#   define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     2000
#   define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             1000  //nik for !!!!! MUSTEK !!!!! (not 2 sec!but 1 sec)
#   define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "2" // Only for display on LCD
#   define SECONDS_BETWEEN_CAN_FRAME_SEND                  "2" // Only for display on LCD
#endif

/*
 * Error beep behavior
 * Overvoltage error cannot be suppressed by a macro!
 * If NO NO_BEEP_ON_ERROR, ONE_BEEP_ON_ERROR or MULTIPLE_BEEPS_WITH_TIMEOUT are activated, we beep forever until error vanishes.
 */
//#define NO_BEEP_ON_ERROR              // If activated, Do not beep on error or timeout.
//#define ONE_BEEP_ON_ERROR             // If activated, only beep once if error was detected.
#define SUPPRESS_CONSECUTIVE_SAME_ALARMS // If activated, recurring alarms are suppressed, e.g. undervoltage alarm tends to come and go in a minute period
#define BEEP_TIMEOUT_SECONDS        10L // 1 minute, Maximum is 254 seconds = 4 min 14 s

#if !defined(NO_BEEP_ON_ERROR) && !defined(ONE_BEEP_ON_ERROR)
#   define MULTIPLE_BEEPS_WITH_TIMEOUT     // If activated, beep for 1 minute if error was detected. Timeout is disabled if debug is active.
#endif

bool sDoAlarmOrTimeoutBeep = false;     // If true, we do an error beep at the end of the loop

#if defined(MULTIPLE_BEEPS_WITH_TIMEOUT)  // Beep one minute
uint8_t sBeepTimeoutCounter = 0;
#endif

#if defined(NO_BEEP_ON_ERROR) && defined(ONE_BEEP_ON_ERROR)
#   error NO_BEEP_ON_ERROR and ONE_BEEP_ON_ERROR are both defined, which makes no sense!
#endif

#define MILLIS_IN_ONE_SECOND 1000L

/*
 * Page button stuff
 *
 * Button at INT0 / D2 for switching LCD pages
 */
#if defined(ARDUINO_AVR_ATmega644)
#   define USE_INT2_FOR_BUTTON_0
#endif

#define USE_BUTTON_0                 // Enable code for 1. button at INT0 / D2
#define BUTTON_DEBOUNCING_MILLIS 100 // With this you can adapt to the characteristic of your button. Default is 50.
#define NO_BUTTON_RELEASE_CALLBACK   // Disables the code for release callback. This saves 2 bytes RAM and 64 bytes program memory.
#include "EasyButtonAtInt01.hpp"

volatile bool sPageButtonJustPressed = false;
void handlePageButtonPress(bool aButtonToggleState);        // The button press callback function sets just a flag.
EasyButton PageSwitchButtonAtPin2(&handlePageButtonPress);  // Button is connected to INT0
#define LONG_PRESS_BUTTON_DURATION_MILLIS   1000
bool sDebugModeActivated = false; // Is activated on long press
void checkButtonPress();

bool readJK_BMSStatusFrame();
void processJK_BMSStatusFrame();
void handleFrameReceiveTimeout();

//#include "HexDump.hpp"
#include "digitalWriteFast.h"

//Software serial for JK-BMS stuff
#if !defined(MAXIMUM_NUMBER_OF_CELLS)
#define MAXIMUM_NUMBER_OF_CELLS     8 // Maximum number of cell info which can be converted. Must be before #include "JK-BMS.hpp".
#endif

#include "JK-BMS.hpp"

//Software serial for JK-BMS request frame sending
#include "SoftwareSerialTX.h"
//Use a 115200 baud software serial for the short request frame.
//If available, we also can use a second hardware serial here :-).
SoftwareSerialTX TxToJKBMS(JK_BMS_TX_PIN);
bool sResponseFrameBytesAreExpected = false;   // If true, request was recently sent so now check for serial input of response frame
uint32_t sMillisOfLastRequestedJKDataFrame = -MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS; // Initial value to start first request immediately
//BMS communication timeout
uint32_t sMillisOfLastReceivedByte = 0;     // For timeout detection
bool sJKBMSFrameHasTimeout;                 // True, as long as BMS timeout persists.
bool sTimeoutJustdetected = false;          // Is set to true at first detection of timeout and reset by beep timeout

//CAN stuff
#if !defined(NO_CAPACITY_35F_EXTENSIONS) // SMA Sunny Island inverters
#define CAPACITY_35F_EXTENSIONS // Add frame 0x35F for total capacity for SMA Sunny Island inverters, which is no problem for Deye inverters.
#endif
#if !defined(NO_CAPACITY_379_EXTENSIONS) // Luxpower SNA inverters
#define CAPACITY_379_EXTENSIONS // Add frame 0x379 for total capacity for Luxpower SNA inverters, which is no problem for Deye inverters.
#endif
#if !defined(NO_BYD_LIMITS_373_EXTENSIONS) // BYD
#define BYD_LIMITS_373_EXTENSIONS // Add frame 0x373 for cell limits as sent by BYD battery, which is no problem for Deye inverters.
#endif
#include "Pylontech_CAN.hpp" // Must be before #include "MCP2515_TX.hpp"
#define CAN_BAUDRATE    500000  // 500 kB
#if !defined(MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE)
// Must be specified before #include "MCP2515_TX.hpp"
#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE  16 // 16 MHz is default for the Arduino CAN bus shield
//#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE   8 // 8 MHz is default for the Chinese breakout board. !!! 8MHz does not work with 500 kB !!!
#endif
#include "MCP2515_TX.hpp"                   // my reduced tx only driver
bool sCANDataIsInitialized = false;         // One time flag, it is never set to false again.
uint32_t sMillisOfLastCANFrameSent = 0;     // For CAN timing
uint32_t sMillisOfLastCANFrameSent_nik = 0;     // For CAN timing (to BLYNK)

/*
 * Optional analytics stuff
 */
#if !defined(NO_ANALYTICS)
#include "JK-BMS_Analytics.hpp"
#endif

/*
 * Optional LCD stuff
 */
#if !defined(USE_NO_LCD)
#include "JK-BMS_LCD.hpp"
#endif

bool sBMSFrameProcessingComplete = false; // True if one status frame was received and processed or timeout happened. Then we can do a sleep at the end of the loop.
bool WeGotNewBMSDataForBLYNK = false; //independent send timing for nik's BLYNK

/*
 * Miscellaneous
 */
#define TIMEOUT_MILLIS_FOR_FRAME_REPLY                  100 // I measured 26 ms between request end and end of received 273 byte result
#if TIMEOUT_MILLIS_FOR_FRAME_REPLY > MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS
#error "TIMEOUT_MILLIS_FOR_FRAME_REPLY must be smaller than MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS to detect timeouts"
#endif
bool sInitialActionsPerformed = false; // Flag to send static info only once after reset.

void processReceivedData();
void printReceivedData();
bool isVCCTooHighSimple();
void handleOvervoltage();

/*
 * Must be after all .hpp includes
 */
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#include "LocalDebugLevelStart.h" // no include "LocalDebugLevelEnd.h" required :-)

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

const char StringStartingCANFailed[] PROGMEM = "Starting CAN failed!";

void MainCycle_StartEvent(){
  #ifdef testmode
		Serial.print("----------V = ");
		Serial.print(fround(voltageV,2));
    Serial.print("   -------A = ");
		Serial.println(fround(currentA,2));
  #endif
  /*
  long curTime = millis();
  if(firstSend){
    firstSend = 0;
    lastSendTime = curTime;
    BAT_EnergyWH += SumDeltaWs/3600;
    SumDeltaWs = 0;
  }else{
    BAT_CurPowerW = SumDeltaWs / (curTime-lastSendTime)*1000; //average
    lastSendTime = curTime;
    BAT_EnergyWH += SumDeltaWs/3600;
    SumDeltaWs = 0;
    BAT_EnergyPercent = BAT_MaxEnergyWH==0 ? 0 : (BAT_EnergyWH / BAT_MaxEnergyWH * 100.0f);

    //calculate average V:
    VArSumTotal = 0;
    for(byte ii=0; ii<VArNumReadings; ii++){
      VArSumTotal += VArReadings[ii];
    }
    AverageV = VArSumTotal / VArNumReadings;

    ////addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(voltageV,2));
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(AverageV,2)); 
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterA,fround(currentA,2));
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_CurPowerW,fround(BAT_CurPowerW,0));
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_EnergyPercent,fround(BAT_EnergyPercent,1));
  }*/
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(JKComputedData.BatteryVoltage10Millivolt/100.0f,2)); 
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterA,fround(JKComputedData.Battery10MilliAmpere/100.0f,2));
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_CurPowerW,fround(BAT_CurPowerW,0));
    //addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_EnergyPercent,fround(sJKFAllReplyPointer->SOCPercent,1));
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
		case VPIN_BAT_SendIntervalSec:
			if(MainCycleInterval == (int)vPinValueFloat || (int)(vPinValueFloat)<2)
				break;
			MainCycleInterval = (int)vPinValueFloat;
			timer.deleteTimer(mainTimerId);
			mainTimerId = timer.setInterval(1000L * MainCycleInterval, MainCycle_StartEvent); //start regularly
			EEPROM_storeValues();
			break;
	
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
  
  EEPROM.update(0,(unsigned char)boardSTATUS);
  EEPROM.update(10,(unsigned char)(MainCycleInterval));

  //EEPROM_WriteAnything(VPIN_BAT_EnergyWH                ,(float)BAT_EnergyWH);
  //EEPROM_WriteAnything(VPIN_BAT_EnergyWH+sizeof(float)*1,(float)BAT_MaxEnergyWH);
  //EEPROM_WriteAnything(VPIN_BAT_EnergyWH+sizeof(float)*2,(float)BAT_EnergyPercent);
}
void EEPROM_restoreValues(){
  int ival;
  EEPROM_ReadAnything(eepromVIAddr,ival);
  if(ival != eepromValueIs){
    EEPROM_storeValues();
    return; //never wrote valid values into eeprom
  }

  boardSTATUS = EEPROM.read(0);
  
  int aNewInterval = EEPROM.read(10);
  if(aNewInterval >=2){
    MainCycleInterval = aNewInterval;
  }
  
 // EEPROM_ReadAnything(VPIN_BAT_EnergyWH                ,BAT_EnergyWH);
  //EEPROM_ReadAnything(VPIN_BAT_EnergyWH+sizeof(float)*1,BAT_MaxEnergyWH);
 // EEPROM_ReadAnything(VPIN_BAT_EnergyWH+sizeof(float)*2,BAT_EnergyPercent);
}

void setup() {
// LED_BUILTIN pin is used as SPI Clock !!!
//    pinModeFast(LED_BUILTIN, OUTPUT);
//    digitalWriteFast(LED_BUILTIN, LOW);
    LED_WS2812_strip.setBrightness(30);
    timer.setInterval(1000L, LED_WS2812_Update);

    EEPROM_restoreValues();
    //timer.setInterval(1000L*3600L, EEPROM_storeValues); //once in 1h remember critical values
    // Initialize CAN bus MCP2515: mode = the masks and filters disabled.
    if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK) //MCP_ANY, MCP_STD, MCP_STDEXT
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


    #if !defined(USE_NO_COMMUNICATION_STATUS_LEDS)
    pinModeFast(BMS_COMMUNICATION_STATUS_LED_PIN, OUTPUT);
    pinModeFast(CAN_COMMUNICATION_STATUS_LED_PIN, OUTPUT);
    #endif

    #if !defined(NO_ANALYTICS)
    pinModeFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN, INPUT_PULLUP);
    #endif

    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

    #if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
    #endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    #if defined(USE_LAYOUT_FOR_644_BOARD)
    JK_INFO_PRINTLN(F("Settings are for Andres 644 board"));
    #endif

    #if defined(ENABLE_MONITORING)
    JK_INFO_PRINTLN(F("Monitoring enabled"));
    #else
    JK_INFO_PRINTLN(F("Monitoring disabled"));
    #endif

    #if defined(NO_CELL_STATISTICS)
    JK_INFO_PRINTLN(F("Cell statistics deactivated"));
    #else
    JK_INFO_PRINTLN(F("Cell statistics activated"));
    #endif

    #if defined(NO_ANALYTICS)
    JK_INFO_PRINTLN(F("Analytics deactivated"));
    #else
    JK_INFO_PRINTLN(F("Analytics activated"));
    readBatteryESRfromEEPROM();
    JK_INFO_PRINT(F("EEPROM BatteryESR="));
    JK_INFO_PRINTLN(sBatteryESRMilliohm);

    findFirstSOCDataPointIndex();
    #if defined(USE_LAYOUT_FOR_644_BOARD)
    JK_INFO_PRINT(F("EEPROM SOC data start index="));
    JK_INFO_PRINT(SOCDataPointsInfo.ArrayStartIndex);
    JK_INFO_PRINT(F(" length="));
    JK_INFO_PRINT(SOCDataPointsInfo.ArrayLength);
    JK_INFO_PRINT(F(", even="));
    JK_INFO_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
    #else
    DEBUG_PRINT(F("EEPROM SOC data start index="));
    DEBUG_PRINT(SOCDataPointsInfo.ArrayStartIndex);
    DEBUG_PRINT(F(" length="));
    DEBUG_PRINT(SOCDataPointsInfo.ArrayLength);
    DEBUG_PRINT(F(", even="));
    DEBUG_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
    #endif
    JK_INFO_PRINTLN(F("Disable ESR compensation pin=" STR(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN)));
    JK_INFO_PRINT(F("Battery ESR compensation for voltage "));
    if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) == LOW) {
        JK_INFO_PRINT(F("dis"));
    } else {
        JK_INFO_PRINT(F("en"));
    }
    JK_INFO_PRINT(F("abled. Specified ESR="));
    JK_INFO_PRINT(sBatteryESRMilliohm);
    JK_INFO_PRINTLN(F("mOhm"));
    #endif

    tone(BUZZER_PIN, 3000, 100);

    #if defined(USE_SERIAL_2004_LCD)
    setupLCD();
    #else
    JK_INFO_PRINTLN(F("LCD code deactivated"));
    #endif

    /*
     * 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX.
     */
    TxToJKBMS.begin(115200);
    Serial.println(F("Serial to JK-BMS started with 115.200 kbit/s!"));
    #if defined(USE_SERIAL_2004_LCD)
    if (sSerialLCDAvailable) {
        myLCD.setCursor(0, 2);
        myLCD.print(F("BMS serial started"));
    }
    #endif

    /*
     * CAN initialization
     */
    if (initializeCAN(CAN_BAUDRATE, MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE, &Serial) == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
        Serial.println(F("CAN started with 500 kbit/s!"));
    #if defined(USE_SERIAL_2004_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("CAN started"));
            delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
        }
    #endif
    } else {
        Serial.print(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
        Serial.println(SPI_CS_PIN);
    #if defined(USE_SERIAL_2004_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
    #  if defined(STANDALONE_TEST)
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    #  else
            delay(4 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
    #  endif
        }
    #endif

    }

    /*
     * Print debug pin info
     */
    #if defined(USE_SERIAL_2004_LCD)
        Serial.println(F("Page switching button is at pin " STR(PAGE_BUTTON_PIN)));
        Serial.println(F("At long press, CAN Info page is entered and additional debug data is printed as long as button is pressed"));
    #else
        Serial.println(F("Debug button is at pin " STR(PAGE_BUTTON_PIN)));
        Serial.println(F("Additional debug data is printed as long as button is pressed"));
    #endif
    Serial.println(F(STR(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) " ms between 2 BMS requests"));
    Serial.println(F(STR(MILLISECONDS_BETWEEN_CAN_FRAME_SEND) " ms between 2 CAN transmissions"));
    #if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
        Serial.println(F("LCD Backlight timeout is " DISPLAY_ON_TIME_STRING));
    #else
        Serial.println(F("No LCD Backlight timeout"));
    #endif
    Serial.println();

    #if defined(USE_SERIAL_2004_LCD)
        printDebugInfoOnLCD();
    #endif

}

void loop() {
    /*delay(1000);
    if (initializeCAN(CAN_BAUDRATE, MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE, &Serial) == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
            Serial.println(F("CAN started with 500 kbit/s!"));
    #if defined(USE_SERIAL_2004_LCD)
            if (sSerialLCDAvailable) {
                myLCD.setCursor(0, 3);
                myLCD.print(F("CAN started"));
                delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
            }
    #endif
        } else {
            Serial.print(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
            Serial.println(SPI_CS_PIN);
    #if defined(USE_SERIAL_2004_LCD)
            if (sSerialLCDAvailable) {
                myLCD.setCursor(0, 3);
                myLCD.print(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
    #  if defined(STANDALONE_TEST)
                delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    #  else
                delay(4 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
    #  endif
            }
    #endif

        }

        return;*/

    timer.run();
    checkReadCAN();
    
    //checkButtonPress();

    //Request status frame every 2 seconds from BMS
    if (millis() - sMillisOfLastRequestedJKDataFrame >= MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
        sMillisOfLastRequestedJKDataFrame = millis(); // set for next check
        //Flush input buffer and send request to JK-BMS
        while (Serial.available()) {
            Serial.read();
        }
        requestJK_BMSStatusFrame(&TxToJKBMS, sDebugModeActivated); // 1.85 ms
        sResponseFrameBytesAreExpected = true; // Enable check for serial input of response
        initJKReplyFrameBuffer();
        sMillisOfLastReceivedByte = millis(); // initialize reply timeout
    }

    //Get reply from BMS and check timeout
    if (sResponseFrameBytesAreExpected) {
        if (Serial.available()) {
            //TURN_BMS_STATUS_LED_ON;
            if (readJK_BMSStatusFrame()) {
                //Frame completely received, now process it
                sBMSFrameProcessingComplete = true;
                WeGotNewBMSDataForBLYNK = true;

                TURN_BMS_STATUS_LED_ON;
                processJK_BMSStatusFrame(); // Process the complete receiving of the status frame and set the appropriate flags
                TURN_BMS_STATUS_LED_OFF;
            }
            //TURN_BMS_STATUS_LED_OFF;
        } else if (millis() - sMillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY) {
            //Here we have requested response frame, but serial was not available fore a longer time => timeout at receiving
            sBMSFrameProcessingComplete = true; // Do frame complete handling like beep anyway
            sResponseFrameBytesAreExpected = false; // Do not try to receive more response bytes

            if (!sJKBMSFrameHasTimeout) {
                //Do this only once per timeout
                sJKBMSFrameHasTimeout = true;
                sTimeoutJustdetected = true; // This forces the beep
                handleFrameReceiveTimeout();
            }
        }
    }

    /*Send CAN frame independently of the period of JK-BMS data requests,
     * but do not send, if frame is currently requested and not completely received and processed.
     * 0.5 MB/s
     * Inverter reply every second: 0x305: 00-00-00-00-00-00-00-00
     * Do not send, if BMS is starting up, the 0% SOC during this time will trigger a deye error beep.
     */
    #if defined(TRACE)
    Serial.print(F("sCANDataIsInitialized="));
    Serial.print(sCANDataIsInitialized);
    Serial.print(F(" BMSIsStarting="));
    Serial.print(JKComputedData.BMSIsStarting);
    Serial.print(F(", sResponseFrameBytesAreExpected="));
    Serial.println(sResponseFrameBytesAreExpected);
    #endif
    if (sCANDataIsInitialized && !JKComputedData.BMSIsStarting && !sResponseFrameBytesAreExpected
            && millis() - sMillisOfLastCANFrameSent >= MILLISECONDS_BETWEEN_CAN_FRAME_SEND) {
        sMillisOfLastCANFrameSent = millis();

        TURN_CAN_STATUS_LED_ON;
        if (sDebugModeActivated) {
            Serial.println(F("Send CAN"));
        }
        sendAllCANFrames(sDebugModeActivated);
        TURN_CAN_STATUS_LED_OFF;
    }

    //nik:
    if (sCANDataIsInitialized 
            && WeGotNewBMSDataForBLYNK
            && !JKComputedData.BMSIsStarting 
            && !sResponseFrameBytesAreExpected
            && millis() - sMillisOfLastCANFrameSent_nik >= (uint32_t)MainCycleInterval*1000L) {
        sMillisOfLastCANFrameSent_nik = millis();

        //TURN_CAN_STATUS_LED_ON;
        if (sDebugModeActivated) {
            Serial.println(F("Send CAN (BLYNK)"));
        }
        
        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterV,fround(JKComputedData.BatteryVoltageFloat,2)); 
        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_VoltmeterA,fround(JKComputedData.BatteryLoadCurrentFloat,2));
        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_CurPowerW,fround(JKComputedData.BatteryLoadPower,0));
        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_EnergyPercent,fround(sJKFAllReplyPointer->SOCPercent,0));

        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_DeltaCellmV,fround((float)JKConvertedCellInfo.DeltaCellMillivolt,0));
        addCANMessage2Queue(CAN_Unit_FILTER_ESPWF | CAN_MSG_FILTER_INF, VPIN_BAT_MaxTemperature,fround(JKComputedData.TemperatureMaximum,0));

        WeGotNewBMSDataForBLYNK = false;
        //TURN_CAN_STATUS_LED_OFF;
    }
        
    /**********************************************************
     * Do this once after each complete status frame or timeout
     **********************************************************/
    if (sBMSFrameProcessingComplete) {
        sDebugModeActivated = false; // reset flag here. It may be set again at start of next loop.

        /*
         * Check for overvoltage
         */
        while (isVCCTooHighSimple()) {
            handleOvervoltage();
        }

        #if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
        if (sSerialLCDAvailable) {
            doLCDBacklightTimeoutHandling();
        }
        #endif

        //Check for BMS alarm flags
        if (sAlarmJustGetsActive || sTimeoutJustdetected) {
            sDoAlarmOrTimeoutBeep = true;
        #if defined(MULTIPLE_BEEPS_WITH_TIMEOUT)
        } else {
            sBeepTimeoutCounter = 0;
        #endif
        }

        //Alarm / beep handling
        #if !defined(NO_BEEP_ON_ERROR)      // Beep enabled

        #  if defined(ONE_BEEP_ON_ERROR)    // Beep once
        if (sDoAlarmOrTimeoutBeep) {
            sAlarmJustGetsActive = false; // disable further alarm beeps
            sTimeoutJustdetected = false; // disable further timeout beeps
        }
        #  endif

        if (sDoAlarmOrTimeoutBeep) {
            sDoAlarmOrTimeoutBeep = false;
        #  if defined(MULTIPLE_BEEPS_WITH_TIMEOUT) // Beep one minute
            sBeepTimeoutCounter++; // incremented at each frame request
            if (sBeepTimeoutCounter == (BEEP_TIMEOUT_SECONDS * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
                JK_INFO_PRINTLN(F("Timeout reached, suppress consecutive error beeps"));
                sAlarmJustGetsActive = false; // disable further alarm beeps
                sTimeoutJustdetected = false; // disable further timeout beeps
            } else
        #  endif
            {
                tone(BUZZER_PIN, 2200);
                tone(BUZZER_PIN, 2200, 50);
                delay(200);
                tone(BUZZER_PIN, 2200, 50);
                delay(200); // to avoid tone interrupts waking us up from sleep
            }
        }
        #endif // NO_BEEP_ON_ERROR

        sBMSFrameProcessingComplete = false; // prepare for next loop

        ///////////////////////////////////////show SOC and Voltage by WS2812 LEDs///////////////////////
        if(!sJKBMSFrameHasTimeout){
            float showPercent = sJKFAllReplyPointer->SOCPercent;
            float showVoltage = JKComputedData.BatteryVoltageFloat-25.0f; //min 25 max 27.6
            if(showVoltage<0.0f){ 
                showVoltage=0.0f; 
            }else{
                if(showVoltage>2.6f)
                    showVoltage=2.6f;
            }
            LED_WS_count = fround( 16.0f*(showPercent/100.0f) ,0);
            LED_WS_color = mWheel8(fround( 255.0f*showVoltage/2.6f ,0)); //rainbow colors 0..255
        }else{
            LED_WS_count = -1;
            LED_WS_color = mMagenta;
        }
    } // if (sBMSFrameProcessingComplete)
}

//Process the complete receiving of the status frame and set the appropriate flags
void processJK_BMSStatusFrame() {
    if (sDebugModeActivated) {
        //Do it once at every debug start
        if (sReplyFrameBufferIndex == 0) {
            Serial.println(F("sReplyFrameBufferIndex is 0"));
        } else {
            Serial.print(sReplyFrameBufferIndex + 1);
            Serial.println(F(" bytes received"));
            printJKReplyFrameBuffer();
        }
        Serial.println();
    }

    if (sJKBMSFrameHasTimeout) {
        // First response frame after timeout :-)
        sJKBMSFrameHasTimeout = false;
        sTimeoutJustdetected = false;
        #if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
        checkAndTurnLCDOn(); // Reason is printed anyway :-)
        #endif
        JK_INFO_PRINTLN(F("Successfully receiving first BMS status frame after BMS communication timeout"));
    }
    processReceivedData();

    if (!sInitialActionsPerformed) {
        //Do initialization once here
        sInitialActionsPerformed = true;
        initializeComputedData();
        Serial.println();
        printJKStaticInfo();
        #if !defined(NO_ANALYTICS)
        initializeAnalytics();
        JK_INFO_PRINTLN();
        #endif
    }

    printReceivedData();
    #if !defined(NO_ANALYTICS)
    writeSOCData();
    #endif
    //Copy complete reply and computed values for change determination
    lastJKReply.SOCPercent = sJKFAllReplyPointer->SOCPercent;
    lastJKReply.AlarmUnion.AlarmsAsWord = sJKFAllReplyPointer->AlarmUnion.AlarmsAsWord;
    lastJKReply.BMSStatus.StatusAsWord = sJKFAllReplyPointer->BMSStatus.StatusAsWord;
    lastJKReply.SystemWorkingMinutes = sJKFAllReplyPointer->SystemWorkingMinutes;
}

//Reads all bytes of the requested frame into buffer and prints errors
//Sets sResponseFrameBytesAreExpected to false, if frame was complete and manages other flags too
//@return true if frame was completely received
bool readJK_BMSStatusFrame() {
    sMillisOfLastReceivedByte = millis();
    uint8_t tReceiveResultCode = readJK_BMSStatusFrameByte();
    if (tReceiveResultCode == JK_BMS_RECEIVE_FINISHED) {
        /*
         * All JK-BMS status frame data received
         */
        sResponseFrameBytesAreExpected = false; // Do not try to receive more response bytes
        return true;

    } else if (tReceiveResultCode != JK_BMS_RECEIVE_OK) {
        /*
         * Error here
         */
        Serial.print(F("Receive error="));
        Serial.print(tReceiveResultCode);
        Serial.print(F(" at index"));
        Serial.println(sReplyFrameBufferIndex);
        sResponseFrameBytesAreExpected = false; // Do not try to receive more response bytes
        sBMSFrameProcessingComplete = true; // In case of error, we do not call processJK_BMSStatusFrame(), so set flag here.
        printJKReplyFrameBuffer();
    }
    return false;
}

/*
 * Here we have requested frame, but serial was not available fore a longer time => timeout at receiving
 * We are called only once per timeout
 */
void handleFrameReceiveTimeout() {
    Serial.print(F("Receive timeout"));

    if (sReplyFrameBufferIndex != 0) {
        // Print bytes received so far
        Serial.print(F(" at ReplyFrameBufferIndex="));
        Serial.print(sReplyFrameBufferIndex);
        if (sReplyFrameBufferIndex != 0) {
            printJKReplyFrameBuffer();
        }
    }
    Serial.println();

    modifyAllCanDataToInactive();

#if defined(USE_SERIAL_2004_LCD)
    /*
     * Switch on LCD and print timeout message once in case of timeout detection
     */
    if (sSerialLCDAvailable) {
#  if !defined(DISPLAY_ALWAYS_ON)
        if (checkAndTurnLCDOn()) {
            Serial.println(F("BMS communication timeout")); // Reason for switching on LCD display
        }
#  endif
        printTimeoutMessageOnLCD();
    }
#endif
}

void processReceivedData() {
    //Set the static pointer to the start of the reply data which depends on the number of cell voltage entries
    //The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize + the variable length cell data 3 bytes per cell, (CellInfoSize is contained in JKReplyFrameBuffer[12])
    sJKFAllReplyPointer = reinterpret_cast<JKReplyStruct*>(&JKReplyFrameBuffer[JK_BMS_FRAME_HEADER_LENGTH + 2
            + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH]]);

    fillJKConvertedCellInfo();
    //Print newline, if SOC changed
    if (sJKFAllReplyPointer->SOCPercent != lastJKReply.SOCPercent) {
        Serial.println();
    }
    fillJKComputedData();

    computeUpTimeString();
    detectAndPrintAlarmInfo(); // UpTimeString is used here

    fillAllCANData(sJKFAllReplyPointer);
    sCANDataIsInitialized = true; // One time flag
}

/*
 * Called exclusively by processJK_BMSStatusFrame()
 */
void printReceivedData() {
#if defined(USE_SERIAL_2004_LCD)
    if (sLCDDisplayPageNumber != JK_BMS_PAGE_CAPACITY_INFO) {
        // Do not interfere with plotter output
        printJKDynamicInfo(); // Prints newline before SOC[%]=
    }
    printBMSDataOnLCD();
#else
    printJKDynamicInfo();
#endif
}

/*
 * Callback handlers for button press
 * Just set flags for evaluation in checkButtonPress(), otherwise readButtonState() may again be false when checkButtonPress() is called
 */
void handlePageButtonPress(bool aButtonToggleState __attribute__((unused))) {
    sPageButtonJustPressed = true;
}

/*
 * Synchronously handle button press (called from loop)
 */
void checkButtonPress() {
#if defined(USE_SERIAL_2004_LCD)
    checkButtonPressForLCD();
#else
    /*
     * Treat Page button as Debug button
     */
    if (sPageButtonJustPressed) {
        sPageButtonJustPressed = false;
    //nik: no debug
    //    sDebugModeActivated = true; // Is set to false in loop
        Serial.println(F("One time debug print just activated"));
    } else if (PageSwitchButtonAtPin2.readDebouncedButtonState()) {
        // Button is still pressed
    //nik: no debug
    //    sDebugModeActivated = true; // Is set to false in loop
    }
#endif // defined(USE_SERIAL_2004_LCD)
}

void handleOvervoltage() {
    #if defined(USE_SERIAL_2004_LCD)
    if (sSerialLCDAvailable) {
    #  if !defined(DISPLAY_ALWAYS_ON)
        if (sSerialLCDIsSwitchedOff) {
            myLCD.backlight();
            sSerialLCDIsSwitchedOff = false;
        }
    #  endif
        myLCD.clear();
        myLCD.setCursor(0, 0);
        myLCD.print(F("VCC overvoltage"));
        myLCD.setCursor(0, 1);
        myLCD.print(F("VCC > 5.25 V"));
    }
    #endif
    // Do it as long as overvoltage happens
    tone(BUZZER_PIN, 1100, 300);
    delay(300);
    tone(BUZZER_PIN, 2200, 1000);
    delay(1000);
}

#if !defined(_ADC_UTILS_HPP)
#include "ADCUtils.h"
/*
 * Recommended VCC is 1.8 V to 5.5 V, absolute maximum VCC is 6.0 V.
 * Check for 5.25 V, because such overvoltage is quite unlikely to happen during regular operation.
 * Raw reading of 1.1 V is 225 at 5 V.
 * Raw reading of 1.1 V is 221 at 5.1 V.
 * Raw reading of 1.1 V is 214 at 5.25 V (+5 %).
 * Raw reading of 1.1 V is 204 at 5.5 V (+10 %).
 * @return true if overvoltage reached
 */
bool isVCCTooHighSimple() {
    ADMUX = ADC_1_1_VOLT_CHANNEL_MUX | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE);
    // ADCSRB = 0; // Only active if ADATE is set to 1.
// ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADIF) | ADC_PRESCALE128); //  128 -> 104 microseconds per ADC conversion at 16 MHz --- Arduino default
// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

    // Get value
    uint16_t tRawValue = ADCL | (ADCH << 8);

    return tRawValue < 1126000 / VCC_OVERVOLTAGE_THRESHOLD_MILLIVOLT; // < 214
}
#endif // _ADC_UTILS_HPP
