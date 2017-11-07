/*********************************************************************
  Project Lynxmotion Phoenix
  Description: Phoenix, control file.
  The control input subroutine for the phoenix software is placed in this file.
  Can be used with V2.0 and above
  Configuration version: V1.0
  Date: 03-10-2017
  Programmer: Frank Friedhof port to PS3 USB/BT, Jeroen Janssen (aka Xan)
               Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...

  Hardware setup: PS3 version

  - CROSS         Toggle walk method

  Walk method 1:
  - Left Stick    Walk/Strafe
  - Right Stick   Rotate

  Walk method 2:
  - Left Stick    Disable
  - Right Stick   Walk/Rotate

  PS3 CONTROLS:
  [Common Controls]
  - Start         Turn on/off the bot
  - L1            Switch control mode (down)
  - R1            Switch control mode (up)
  - Square        Toggle Balance mode
  - Triangle      Move body to 35 mm from the ground (walk pos)
  and back to the ground
  - D-Pad up      Body up 10 mm
  - D-Pad down    Body down 10 mm
  - D-Pad left    decrease speed with 50mS
  - D-Pad right   increase speed with 50mS

  [Walk Controls]
  - select        Switch gaits
  - Left Stick    (Walk mode 1) Walk/Strafe
                  (Walk mode 2) Disable
  - Right Stick   (Walk mode 1) Rotate,
                  (Walk mode 2) Walk/Rotate
  - L2            Toggle Double gait travel speed
  - R2            Toggle Double gait travel length

  [Shift Controls]
  - Left Stick    Shift body X/Z
  - Right Stick   Shift body Y and rotate body Y

  [Rotate Controls]
  - Left Stick    Rotate body X/Z
  - Right Stick   Rotate body Y

  [Single leg Controls]
  - select        Switch legs
  - Left Stick    Move Leg X/Z (relative)
  - Right Stick   Move Leg Y (absolute)
  - R2            Hold/release leg position
*********************************************************************/

// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#ifdef USE_PS3BT
#include <PS3BT.h>
#include <usbhub.h>
#else
#include <PS3USB.h>
#endif

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
//#define GPPLAYERMODE      4

// The deadzone for the analog input from the remote
#define cTravelDeadZone   4

// How many times through the loop will we go before shutting off robot?
#define MAXPS3ERRORCNT    5

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
//PS2X ps2x; // create PS2 Controller Class
USB Usb;
#ifdef USE_PS3BT
extern PS3BT ps3x;
#else
PS3USB ps3x(&Usb); // This will just create the instance
#endif

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// LiquidCrystal_I2C lcd(0x27, 20, 4);

//#define DBGTerminal lcd
#define DBGTerminal DBGSerial

// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 


static short    g_BodyYOffset; 
static short    g_sPS3ErrorCnt;
static short    g_BodyYShift;
static char     ControlMode;
static bool     DoubleHeightOn;
static bool     DoubleTravelOn;
static bool     WalkMethod;
byte            GPSeq;              // Number of the sequence
short           g_sGPSMController;  // What GPSM value have we calculated. 0xff - Not used yet

// some external or forward function references.
extern void PS3TurnRobotOff(void);

/*******************************************************************************
 * This is the function that is called by the Main program to initialize
 * the input controller, which in this case is the PS2 controller
 * process any commands.
 ******************************************************************************/
void InputController::Init(void) {
  int error;

  //error = ps3x.config_gamepad(57, 55, 56, 54);  // Setup gamepad (clock, command, attention, data) pins
  //error = ps3x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins
  if (Usb.Init() == -1) {
    DBGSerial.println(F("\r\nOSC did not start"));
    while (1); //halt
  }
  DBGSerial.println(F("\r\nPS3 USB/Bluetooth Library Started"));

#ifdef DBGSerial
	DBGSerial.print(F("PS3 Init: "));
	DBGSerial.println(error, DEC);
#endif
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_sPS3ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.

//   lcd.begin();   // initialize the lcd for 20 chars 4 lines, turn on backlight
//   lcd.backlight();   // light ON
//   lcd.clear();
//   lcd.setCursor(0,0);
//   lcd.print(F("PS3 controller"));
//   lcd.setCursor(0,1);
//   lcd.print(F("display"));
//   lcd.autoscroll();
}  // Init

/*******************************************************************************
 * This function is called by the main code to tell us when it is about to
 * do a lot of bit-bang outputs and it would like us to minimize any interrupts
 * that we do while it is active.
 ******************************************************************************/
void InputController::AllowControllerInterrupts(boolean fAllow) {
  // We don't need to do anything...
}

/*******************************************************************************
 * This is The main code to input function to read inputs from the PS3 and then
 * process any commands.
 ******************************************************************************/
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
#endif  // OPT_DYNAMIC_ADJUST_LEGS


/**/
void InputController::ControlInput(void) {
  // Then try to receive a packet of information from the PS3.
  boolean fAdjustLegPositions = false;
  Usb.Task();

  if (ps3x.PS3Connected || ps3x.PS3NavigationConnected) {
  
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;
#endif  // OPT_DYNAMIC_ADJUST_LEGS

    // In an analog mode so should be OK...
    g_sPS3ErrorCnt = 0;    // clear out error count...

//     int roll, pitch;
//     roll  = ps3x.getAngle(Roll) - 180;
//     pitch = ps3x.getAngle(Pitch) - 180;
//     g_InControlState.TurretRotAngle1  = constrain( roll, -90, 90);
//     g_InControlState.TurretTiltAngle1 = constrain( pitch, -45, 45);
//     Serial.print(F("\r\nPitch: "));
//     Serial.print(g_InControlState.TurretTiltAngle1);
//     Serial.print(F("\tRoll: "));
//     Serial.print(g_InControlState.TurretRotAngle1);

    if (ps3x.getButtonClick(START)) {
      if (g_InControlState.fRobotOn) {
        DBGTerminal.println(F("Robot turned OFF."));
        PS3TurnRobotOff();
      } 
      else {
        //Turn on
        DBGTerminal.println(F("Robot turned ON."));
        g_InControlState.fRobotOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fRobotOn) {
      bool cm_switched = false;
      // switch through modes
      if (ps3x.getButtonClick(R1)) {
        ControlMode++;
        if (ControlMode > SINGLELEGMODE)
          ControlMode = WALKMODE;
        cm_switched = true;
      }
      if (ps3x.getButtonClick(L1)) {
        ControlMode--;
        if (ControlMode < WALKMODE)
          ControlMode = SINGLELEGMODE;
        cm_switched = true;
      }

      if (cm_switched) {
        MSound( 1, 50, 2000);
        cm_switched = false;

        if (ControlMode == WALKMODE) {
          DBGTerminal.println(F("Walk mode"));
          g_InControlState.SelectedLeg = 255;
        }
        else if (ControlMode == TRANSLATEMODE) {
          DBGTerminal.println(F("Translate mode"));
        }
        else if (ControlMode == ROTATEMODE) {
          DBGTerminal.println(F("Rotate mode"));
        }
#ifdef OPT_SINGLELEG
        else if (ControlMode == SINGLELEGMODE) {
          DBGTerminal.println(F("Single leg mode"));
          if (abs(g_InControlState.TravelLength.x)   < cTravelDeadZone &&
              abs(g_InControlState.TravelLength.z)   < cTravelDeadZone &&
              abs(g_InControlState.TravelLength.y*2) < cTravelDeadZone ) {
            if (g_InControlState.SelectedLeg == 255)  // Select leg if none is selected
              g_InControlState.SelectedLeg = cRR;     // Startleg
          }
        }
#endif  // OPT_SINGLELEG
      }

      // [Common functions]
      // Switch Balance mode on/off 
      if (ps3x.getButtonClick(SQUARE)) {
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500);
          DBGTerminal.println(F("Balance mode ON"));
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
          DBGTerminal.println(F("Balance mode OFF"));
        }
      }

      // Stand up, sit down  
      if (ps3x.getButtonClick(TRIANGLE)) {
        if (g_BodyYOffset>0) 
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
        DBGTerminal.print(F("BodyYOffset : "));
        DBGTerminal.println(g_BodyYOffset);
      }

      if (ps3x.getButtonClick(UP)) {
        g_BodyYOffset += 10;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
        DBGTerminal.print(F("BodyYOffset : "));
        DBGTerminal.println(g_BodyYOffset);
      }

      if (ps3x.getButtonClick(DOWN) && g_BodyYOffset) {
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        DBGTerminal.print(F("BodyYOffset : "));
        DBGTerminal.println(g_BodyYOffset);
      }

      if (ps3x.getButtonClick(RIGHT)) {
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
        DBGTerminal.print(F("SpeedControl : "));
        DBGTerminal.println(g_InControlState.SpeedControl);
      }

      if (ps3x.getButtonClick(LEFT)) {
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
        DBGTerminal.print(F("SpeedControl : "));
        DBGTerminal.println(g_InControlState.SpeedControl, DEC);
      }
      
      // We are optionally going to allow the user to modify the Initial Leg positions, when they
      // press the L3 button.
      byte lx = ps3x.getAnalogHat(LeftHatX);
      byte ly = ps3x.getAnalogHat(LeftHatY);

#ifdef OPT_DYNAMIC_ADJUST_LEGS
      if (ps3x.getButtonPress(CIRCLE)) {
        sLegInitXZAdjust    = (int(lx) - 128) / 10;        // play with this.
        sLegInitAngleAdjust = (int(ly) - 128) / 8;
        lx = 0;
        ly = 0;
      }
#endif  // OPT_DYNAMIC_ADJUST_LEGS

      // [Walk functions]
      if (ControlMode == WALKMODE) {
        // Switch gaites
        if (ps3x.getButtonClick(SELECT) &&
            abs(g_InControlState.TravelLength.x)   < cTravelDeadZone && // No movement
            abs(g_InControlState.TravelLength.z)   < cTravelDeadZone &&
            abs(g_InControlState.TravelLength.y*2) < cTravelDeadZone) {
          // Go to the next gait...
          g_InControlState.GaitType++;
          // Make sure we did not exceed number of gaits...
          if (g_InControlState.GaitType < NUM_GAITS) {
            MSound( 1, 50, 2000); 
          } 
          else {
            MSound(2, 50, 2000, 50, 2250); 
            g_InControlState.GaitType = 0;
          }
          GaitSelect();
        }

        // Double leg lift height
        if (ps3x.getButtonClick(L2)) {
          MSound( 1, 50, 2000); 
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 80;
          else
            g_InControlState.LegLiftHeight = 50;
          DBGTerminal.print(F("Gait level "));
          DBGTerminal.println(g_InControlState.LegLiftHeight);
        }

        // Double Travel Length
        if (ps3x.getButtonClick(R2)) {
          MSound(1, 50, 2000); 
          DoubleTravelOn = !DoubleTravelOn;
          DBGTerminal.print(F("Gait speed "));
          DBGTerminal.println(DoubleTravelOn ? F("double") : F("normal"));
        }

        // Switch between Walk method 1 && Walk method 2
        if (ps3x.getButtonClick(CROSS)) {
          MSound(1, 50, 2000); 
          WalkMethod = !WalkMethod;
          DBGTerminal.print(F("Walk method "));
          DBGTerminal.println(WalkMethod ? F("1") : F("2"));
        }

        // Walking
        if (WalkMethod)
          // (Walk Methode) 
          // Right Stick Up/Down  
          g_InControlState.TravelLength.z = (ps3x.getAnalogHat(RightHatY) - 128);
        else {
          g_InControlState.TravelLength.x = -(lx - 128);
          g_InControlState.TravelLength.z =  (ly - 128);
        }

        if (!DoubleTravelOn) {
          // (Double travel length)
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x / 2;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z / 2;
        }

        g_InControlState.TravelLength.y = -(ps3x.getAnalogHat(RightHatX) - 128)/4; //Right Stick Left/Right 
      }

      // [Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x =  (lx - 128) / 2;
        g_InControlState.BodyPos.z = -(ly - 128) / 3;
        g_InControlState.BodyRot1.y = (ps3x.getAnalogHat(RightHatX) - 128) * 2;
        g_BodyYShift = (-(ps3x.getAnalogHat(RightHatY) - 128) / 2);
      }

      // [Rotate functions]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (ly - 128);
        g_InControlState.BodyRot1.y = (ps3x.getAnalogHat(RightHatX) - 128) * 2;
        g_InControlState.BodyRot1.z = (lx - 128);
        g_BodyYShift = (-(ps3x.getAnalogHat(RightHatY) - 128) / 2);
      }

      // [Single leg functions]

#ifdef OPT_SINGLELEG
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ps3x.getButtonClick(SELECT)) {
          MSound(1, 50, 2000); 
          if (g_InControlState.SelectedLeg < (CNT_LEGS-1))
            g_InControlState.SelectedLeg++;
          else
            g_InControlState.SelectedLeg = 0;

          DBGTerminal.print(F("Selected leg : "));
          DBGTerminal.println(g_InControlState.SelectedLeg, DEC);
        }

        // Left Stick Right/Left
        g_InControlState.SLLeg.x = (lx - 128) / 2;
        // Right Stick Up/Down
        g_InControlState.SLLeg.y = (ps3x.getAnalogHat(RightHatY) - 128) / 2;
        // Left Stick Up/Down
        g_InControlState.SLLeg.z = (ly - 128) / 2;

        // Hold single leg in place
        if (ps3x.getButtonClick(R2)) {
          MSound(1, 50, 2000);  
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }
#endif  // OPT_SINGLELEG

      // Calculate walking time delay
      g_InControlState.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(ps3x.getAnalogHat(RightHatX) - 128));
    }

    // Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when
      // we have finished any previous adjustment

      if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        // Add on current length to our adjustment...
        sLegInitXZAdjust += GetLegsXZLength();
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) 
            RotateLegInitAngles(sLegInitAngleAdjust);
        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    
#endif  // OPT_DYNAMIC_ADJUST_LEGS
    
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
  } 
  else {
    // We may have lost the PS2... See what we can do to recover...
    if (g_sPS3ErrorCnt < MAXPS3ERRORCNT)
      g_sPS3ErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (g_InControlState.fRobotOn)
      PS3TurnRobotOff();
//    ps3x.reconfig_gamepad();
  }

  if (ps3x.getButtonClick(PS)) {
    Serial.println(F("\r\nSwitching robot and controller OFF."));
    PS3TurnRobotOff();
    delay(1000);
    ps3x.disconnect();
    MSound(2, 50, 2000, 50, 2250, 50, 1000, 50, 500, 50, 250);
  }
}  // ControlInput


/*
 * PS3TurnRobotOff - code used couple of places so save a little room...
 */
void PS3TurnRobotOff(void) {
  //Turn off
  g_InControlState.TurretRotAngle1  = 0;
  g_InControlState.TurretTiltAngle1 = 0;
  g_InControlState.BodyPos.x        = 0;
  g_InControlState.BodyPos.y        = 0;
  g_InControlState.BodyPos.z        = 0;
  g_InControlState.BodyRot1.x       = 0;
  g_InControlState.BodyRot1.y       = 0;
  g_InControlState.BodyRot1.z       = 0;
  g_InControlState.TravelLength.x   = 0;
  g_InControlState.TravelLength.z   = 0;
  g_InControlState.TravelLength.y   = 0;
  g_BodyYOffset                     = 0;
  g_BodyYShift                      = 0;
#ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg      = 255;
#endif  
  g_InControlState.fRobotOn         = 0;

  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}  // PS3TurnRobotOff
