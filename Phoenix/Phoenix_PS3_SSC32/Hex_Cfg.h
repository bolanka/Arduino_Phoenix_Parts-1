/*********************************************************************
 * Project: Phoenix Hexapod
 * 
 * Description: custom Phoenix HW, configuration file.
 * 
 * All Hardware connections (excl controls) and body dimensions 
 * are configurated in this file. Can be used with V2.0 and above
 * 
 * Configuration version: V1.0
 * Date: Nov 1, 2009
 * Programmer: Kurt (aka KurtE), Frank Friedhof
 * 
 * Hardware setup: Arduino Mega 2560, SSC32U, Kuman MG996R servos, 
 *                 PS3 controller with USB or BT
 * 
 */

#ifndef HEX_CFG_PHOENIX3_H
#define HEX_CFG_PHOENIX3_H

/*
 * [CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
 * Define other optional compnents to be included or not...
 */
#define OPT_TERMINAL_MONITOR  
//#define OPT_TERMINAL_MONITOR_IC
//#define OPT_DUMP_EEPROM
#define OPT_DYNAMIC_ADJUST_LEGS

#ifdef OPT_TERMINAL_MONITOR       // turning off terminal monitor will turn these off as well...
#define OPT_SSC_FORWARDER         // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif  // OPT_TERMINAL_MONITOR

//#define OPT_GPPLAYER            // not with SSC32U?
//#define DEBUG_IK
#define OPT_SINGLELEG
#define DISPLAY_GAIT_NAMES


/* 
 * [SERIAL CONNECTIONS]
 */
#define DBGSerial Serial
#define SSCSerial Serial1

#define USE_PS3BT
#ifdef USE_PS3BT
// pls replace this MAC address by your own
#define PS3_BT_MAC 0x00, 0x15, 0x83, 0xED, 0xBD, 0x19
#endif

#define cDBG_BAUD        38400    // terminal BAUD rate
#define cSSC_BAUD        38400    // SSC32 BAUD rate
//#define  cSSC_BINARYMODE      0    // Define if your SSC-32 card supports binary mode.
//#define DEBUG_IOPINS


#define USEPS3

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// Phoenix
//==================================================================================================================================
#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.


//--------------------------------------------------------------------
//[Botboarduino Pin Numbers]
#define SOUND_PIN   40        // Arduino JR pin number
//#define cVoltagePin  "VA\r"   // using SSC32 sensor IF
#define cVoltagePin A6        // using Arduino analog input A6 for voltage monitor
#define cCurrentPin A7        // using Arduino analog input A7 for current monitor

/*
 * Voltage divider and voltages
 * 
 * remarks: actually I'm using Kuman MG996R servos, datasheet says
 *          will work 4.8-7.2 V, but many servos were killed with
 *          6 cell battery.
 */
#ifdef cVoltagePin
#define CSSC32_VBUF   1    // hown may voltages to read
#define CVADR1      324    // 32k4 Ohms, VD Resistor 1
#define CVADR2       95    // 9k5 Ohms, VD Resistor 2
#define CVREF       500    // 5.0 V
// 5 cell NiMH battery
#define cTurnOffVol 400    // 4.7 V
#define cTurnOnVol  500    // 5.5 V
// 6 cell NiMH battery
//#define cTurnOffVol 650    // 6.5 V
//#define cTurnOnVol  720    // 7.2 V
#endif  // cVoltagePin

#ifdef cVoltagePin == A0
extern uint16_t GetBatteryVoltage(void);
#endif

#ifdef cCurrentPin == A7
extern int16_t GetTotalCurrent(void);
#endif

/*
 * [SSC PIN NUMBERS]
 * 
 * I did it a little different, because of short Tibia cable
 */
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal
#define cRRFemurPin     1   //Rear Right leg Hip Vertical
#define cRRTibiaPin     2   //Rear Right leg Knee

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal
#define cRMFemurPin     5   //Middle Right leg Hip Vertical
#define cRMTibiaPin     6   //Middle Right leg Knee

#define cRFCoxaPin      12   //Front Right leg Hip Horizontal
#define cRFFemurPin     13   //Front Right leg Hip Vertical
#define cRFTibiaPin     14   //Front Right leg Knee

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal
#define cLRFemurPin     17   //Rear Left leg Hip Vertical
#define cLRTibiaPin     18   //Rear Left leg Knee

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal
#define cLMFemurPin     21   //Middle Left leg Hip Vertical
#define cLMTibiaPin     22   //Middle Left leg Knee

#define cLFCoxaPin      28   //Front Left leg Hip Horizontal
#define cLFFemurPin     29   //Front Left leg Hip Vertical
#define cLFTibiaPin     30   //Front Left leg Knee

#define cTurretRotPin   25
#define cTurretTiltPin  27


/*
 * [MIN/MAX ANGLES], 1/10 degrees
 */
#define cRRCoxaMin1  -260  // Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1   740
#define cRRFemurMin1 -900
#define cRRFemurMax1  900
#define cRRTibiaMin1 -900
#define cRRTibiaMax1  770

#define cRMCoxaMin1  -530  // Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1   530
#define cRMFemurMin1 -900
#define cRMFemurMax1  900
#define cRMTibiaMin1 -900
#define cRMTibiaMax1  770

#define cRFCoxaMin1  -580  // Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1   740
#define cRFFemurMin1 -900
#define cRFFemurMax1  900
#define cRFTibiaMin1 -900
#define cRFTibiaMax1  770

#define cLRCoxaMin1  -740  // Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1   260
#define cLRFemurMin1 -900
#define cLRFemurMax1  900
#define cLRTibiaMin1 -770
#define cLRTibiaMax1  900

#define cLMCoxaMin1  -530  // Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1   530
#define cLMFemurMin1 -900
#define cLMFemurMax1  900
#define cLMTibiaMin1 -770
#define cLMTibiaMax1  900

#define cLFCoxaMin1  -740  // Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1   580
#define cLFFemurMin1 -950
#define cLFFemurMax1  900
#define cLFTibiaMin1 -770
#define cLFTibiaMax1  900


/*
 * [LEG DIMENSIONS], unit : mm
 */
#define cXXCoxaLength     26                // This is for Phoenix legs
#define cXXFemurLength    76
#define cXXTibiaLength    106

#define cRRCoxaLength     cXXCoxaLength	    // Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength	    // Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength	    // Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength	    // Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength	    // Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength	    // Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength


/*
 * [BODY DIMENSIONS], unit : mm, 1/10 °
 * initial coxa angles
 * coxa offsets from body middle X, Z coordinates
 */
#define cRRCoxaAngle1 -600   // Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0   // Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1  600   // Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1 -600   // Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0   // Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1  600   // Default Coxa setup angle, decimals = 1

#define cRROffsetX     -43   // Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      82   // Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX     -63   // Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ       0   // Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX     -43   // Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ     -82   // Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      43   // Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      82   // Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX      63   // Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ       0   // Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX      43   // Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ     -82   // Distance Z from center of the body to the Left Front coxa

#define MAX_BODY_Y     130   // maximum body height


/*
 * [START POSITIONS FEET]
 */
#define cHexInitXZ      105               // TBD: check this
#define CHexInitXZCos60  53               // COS(60) = .5
#define CHexInitXZSin60  91               // sin(60) = .866
#define CHexInitY        35               // TBD: check this, changed from 25

// Start positions of the Right Rear leg
#define cRRInitPosX     CHexInitXZCos60
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

// Start positions of the Right Middle leg
#define cRMInitPosX     cHexInitXZ
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

// Start positions of the Right Front leg
#define cRFInitPosX     CHexInitXZCos60
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ    -CHexInitXZSin60

// Start positions of the Left Rear leg
#define cLRInitPosX     CHexInitXZCos60
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

// Start positions of the Left Middle leg
#define cLMInitPosX     cHexInitXZ
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

// Start positions of the Left Front leg
#define cLFInitPosX     CHexInitXZCos60
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ    -CHexInitXZSin60

// turret
#define cTurretRotInit  0
#define cTurretTiltInit 0


/********************************************************************/

/*
 * some servos have to be inverted, for my Phoenix.
 */
#define cRRCoxaInv 0
#define cRMCoxaInv 0
#define cRFCoxaInv 0
#define cLRCoxaInv 1
#define cLMCoxaInv 1
#define cLFCoxaInv 1
#define cRRFemurInv 0
#define cRMFemurInv 0
#define cRFFemurInv 0
#define cLRFemurInv 1
#define cLMFemurInv 1
#define cLFFemurInv 1
#define cRRTibiaInv 0
#define cRMTibiaInv 0
#define cRFTibiaInv 0
#define cLRTibiaInv 1
#define cLMTibiaInv 1
#define cLFTibiaInv 1

#endif CFG_HEX_H



