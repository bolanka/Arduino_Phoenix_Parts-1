/*******************************************************************************
 * Project Lynxmotion Phoenix
 * Description: Phoenix software
 * Software version: V2.0
 * Date: 29-10-2009
 * Programmer: Frank Friedhof, PS3 port, some clean up, Arduino Mega, custom Phoenix HW
 *             Jeroen Janssen [aka Xan]
 *             Kurt Eckhardt(KurtE) converted to C and Arduino
 *             Kåre Halvorsen aka Zenta - Makes everything work correctly!
 *
 * source: https://github.com/KurtE/Arduino_Phoenix_Parts
 *         https://github.com/davidhend/Hexapod
*******************************************************************************/

/*
 * Header Files
 */
#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif

#include "Hex_CFG.h"
#include <Phoenix.h>
#include <Phoenix_Input_PS3.h>

#ifdef USE_PS3BT
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS3BT ps3x(&Btd, PS3_BT_MAC);
#endif

#include <Phoenix_Driver_SSC32.h>

// I've (mis)used this to call the controller a few times more
// else all inputs will be delayed by 2...3 seconds, when using BT
#ifdef USE_PS3BT
#ifdef DoBackgroundProcess()
#undef DoBackgroundProcess()
#endif
extern void DoBackgroundProcess();
#endif

#include <Phoenix_Code.h>

#ifdef USE_PS3BT
void DoBackgroundProcess() {
  if (!g_fLowVoltageShutdown) {
    //    DebugWrite(A0, HIGH);
    g_InputController.ControlInput();
    //    DebugWrite(A0, LOW);
  }
}  // DoBackgroundProcess
#endif

