/*
 * Header Files
 */
#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif

#include "Hex_Cfg.h"

/*
 * GetBatteryVoltage - Maybe should try to minimize when this is called
 * as it uses the serial port... Maybe only when we are not interpolating
 * or if maybe some minimum time has elapsed...
 */
#ifdef cVoltagePin

// use standard settings, if not already defined by user
#ifndef CVADR1
#define CVADR1      30  // VD Resistor 1 - reduced as only need ratio... 30K and 10K
#define CVADR2      10  // VD Resistor 2
#endif


uint16_t GetBatteryVoltage(void) {
//  static uint16_t  g_awVoltages[] = {0,0,0,0,0,0,0,0};
  static uint16_t  g_awVoltages[] = {0,0};
  static uint16_t  g_wVoltageSum = 0;
  static byte  g_iVoltages = 0;
  #define cNUM_VAVG_BUF sizeof(g_awVoltages)/sizeof(uint16_t)

  g_iVoltages = (++g_iVoltages) & (cNUM_VAVG_BUF-1);  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
  return ((long)((long)g_wVoltageSum*CVREF*(CVADR1+CVADR2))/(long)(cNUM_VAVG_BUF*1024*(long)CVADR2));  
#else
  return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));  
#endif
}  // GetBatteryVoltage
#endif  // cVoltagePin


#ifdef cCurrentPin
int16_t GetTotalCurrent(void) {
  static int16_t  g_awCurrents[] = {0,0,0,0,0,0,0,0};
  static int16_t  g_wCurrentSum = 0;
  static byte  g_iCurrents = 0;
  #define cNUM_CAVG_BUF sizeof(g_awCurrents)/sizeof(int16_t)

  g_iCurrents = (++g_iCurrents) & (cNUM_CAVG_BUF-1);  // setup index to our array...
  g_wCurrentSum -= g_awCurrents[g_iCurrents];
  // 512 is the virtual GND/0 amps, because the sensor can measure in both directions
  g_awCurrents[g_iCurrents] = analogRead(cCurrentPin) - 512;
  g_wCurrentSum += g_awCurrents[g_iCurrents];

  return int16_t(int32_t(int32_t(g_wCurrentSum) * 20 * 1000) / int32_t(cNUM_CAVG_BUF*1024));
}  // GetTotalCurrent
#endif  // cCurrentPin

