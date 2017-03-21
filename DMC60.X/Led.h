/************************************************************************/
/*									*/
/*  Led.h - Motor Controller Led Functions                              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  configure and control the Motor Controller LEDs.                    */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/15/2016 (MichaelA): created			                */
/*  04/21/2016 (MichaelA): added additional functions for configuring   */
/*      the LEDs to display different patterns and cleaned up code      */
/*  11/02/2016 (MichaelA): added LedRestoreCalStart() and               */
/*      LedRestoreCalComplete() for setting flags that allow the LEDs   */
/*      to inform the user when the servo calibration constants have    */
/*      been restored. Note:  LedTick() was also modified               */
/*									*/
/************************************************************************/

#if !defined(_LED_INC)
#define	_LED_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define macros used to turn on and off the individual LEDs
** via their enable signals. Please note that an LED will remain
** off if the dutcy cycle associated with it's color is set to 0
** regardless of it's current enable state.
*/
#define LED_R1_ON   trsLedR1 &= ~(1 << bnLedR1);
#define LED_R2_ON   trsLedR2 &= ~(1 << bnLedR2);
#define LED_R3_ON   trsLedR3 &= ~(1 << bnLedR3);
#define LED_R4_ON   trsLedR4 &= ~(1 << bnLedR4);
#define LED_G1_ON   trsLedG1 &= ~(1 << bnLedG1);
#define LED_G2_ON   trsLedG2 &= ~(1 << bnLedG2);
#define LED_G3_ON   trsLedG3 &= ~(1 << bnLedG3);
#define LED_G4_ON   trsLedG4 &= ~(1 << bnLedG4);
#define LED_B1_ON   trsLedB1 &= ~(1 << bnLedB1);
#define LED_B2_ON   trsLedB2 &= ~(1 << bnLedB2);
#define LED_B3_ON   trsLedB3 &= ~(1 << bnLedB3);
#define LED_B4_ON   trsLedB4 &= ~(1 << bnLedB4);

#define LED_R1_OFF   trsLedR1 |= (1 << bnLedR1);
#define LED_R2_OFF   trsLedR2 |= (1 << bnLedR2);
#define LED_R3_OFF   trsLedR3 |= (1 << bnLedR3);
#define LED_R4_OFF   trsLedR4 |= (1 << bnLedR4);
#define LED_G1_OFF   trsLedG1 |= (1 << bnLedG1);
#define LED_G2_OFF   trsLedG2 |= (1 << bnLedG2);
#define LED_G3_OFF   trsLedG3 |= (1 << bnLedG3);
#define LED_G4_OFF   trsLedG4 |= (1 << bnLedG4);
#define LED_B1_OFF   trsLedB1 |= (1 << bnLedB1);
#define LED_B2_OFF   trsLedB2 |= (1 << bnLedB2);
#define LED_B3_OFF   trsLedB3 |= (1 << bnLedB3);
#define LED_B4_OFF   trsLedB4 |= (1 << bnLedB4);

/* Define macros for setting and getting the Red, Green, and Blue LED PWM duty
** cycles.
*/
#if defined(DEAD)
#define SetRedDTC(dtc)      OC1R = (dtc << 5)
#define SetGreenDTC(dtc)    OC2R = (dtc << 5)
#define SetBlueDTC(dtc)     OC3R = (dtc << 5)
#define GetRedDTC()         (OC1R >> 5)
#define GetGreenDTC()       (OC2R >> 5)
#define GetBlueDTC()        (OC3R >> 5)
#else
#define SetRedDTC(dtc)      OC1R = (dtc << 4)
#define SetGreenDTC(dtc)    OC2R = (dtc << 4)
#define SetBlueDTC(dtc)     OC3R = (dtc << 4)
#define GetRedDTC()         (OC1R >> 4)
#define GetGreenDTC()       (OC2R >> 4)
#define GetBlueDTC()        (OC3R >> 4)
#endif

/* Define macro used to convert milliseconds to ticks.
*/
#define LedMsToTicks(tms)   (tms << 1)

/* Define constants that identify different colors which may be used when
** calling the functions that accept a color parameter.
*/
#define colorBlack      0
#define colorRed        1
#define colorGreen      2
#define colorBlue       3
#define colorYellow     4
#define colorFuchsia    5
#define colorCyan       6
#define colorOrange     7
#define colorWhite      8

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

/* Define a structure that can be used to keep track of the on
** and off state and duty cycles for all LEDs. This is the on/off state
** that is currently applied to the hardware or a future on/off state that
** will be applied to the physical hardware.
*/
typedef struct {
    union {
        struct {
            unsigned R1:1;
            unsigned R2:1;
            unsigned R3:1;
            unsigned R4:1;
            unsigned G1:1;
            unsigned G2:1;
            unsigned G3:1;
            unsigned G4:1;
            unsigned B1:1;
            unsigned B2:1;
            unsigned B3:1;
            unsigned B4:1;
            unsigned :4;
        };
        WORD leds;
    };
    WORD dtcRed;
    WORD dtcGreen;
    WORD dtcBlue;
} LEDS;

typedef struct {
    LEDS    leds;       // initial physical LED on/off state and duty cycles
    WORD    ctickDtc;   // number of ticks to remain at the current duty cycle
    WORD    ctickSt;    // number of ticks to remaining in this state

    union {
        struct {
            unsigned fGlowR:1; // increase or decrease Red LED duty cycle while in state 0 or 1 after ctickDtc
            unsigned fGlowG:1; // increase or decrease Green LED duty cycle while in state 0 or 1 after ctickDtc
            unsigned fGlowB:1; // increase or decrease Blue LED duty cycle while in state 0 or 1 after ctickDtc
            unsigned fHalfDtcR:1; // use half of the normal Red duty cycle when an increase or decrease occurs
            unsigned fHalfDtcG:1; // use half of the normal Green duty cycle when an increase or decerase occurs
            unsigned fHalfDtcB:1; // use half of the normal Blue duty cycle when an increase or decrease occurs
        };
        WORD    fsFlags;
    };
} LEDOOST;

typedef struct {
    union {
        struct {
            unsigned fInitComplete:1;
            unsigned fCalStart:1;
            unsigned fCalSuccess:1;
            unsigned fCalFailed:1;
            unsigned fDispBC:1;
            unsigned fRestoreCalStart:1;
            unsigned fRestoreCalComplete:1;
            unsigned :9;
        };
        WORD    fsFlags;
    };
} LEDFLAGS;

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    LedTick();
void    LedInit();
void    LedSet(LEDS* pledsSet);
void    LedGet(LEDS* pledsGet);

void    LedAllBlink(BYTE colorDisp, WORD dtcMaster, WORD tmsOn);
void    LedAllSolid(BYTE colorDisp, WORD dtcMaster);

void    LedAlternateBlink(BYTE colorTop, BYTE colorBottom, WORD dtcMaster, WORD tmsOn, WORD tmsDelay);
void    LedCircularBlink(BYTE colorDisp, WORD dtcMaster, WORD tmsOn, BOOL fCC);
void    LedCircularBlinkAdjustRate(WORD tmsOn);
void    LedCircularBlinkAdjustColor(BYTE colorDisp, WORD dtcMaster);
void    LedOppositeBlink(BYTE colorTop, BYTE colorBottom, WORD dtcMaster, WORD tmsOn, WORD tmsDelay);
void    LedProgressiveGlow(BYTE colorDisp, WORD tmsStep, WORD tmsOn, WORD tmsOff, BOOL fUpDown);

void    LedStrobe(WORD dtcR, WORD dtcG, WORD dtcB, WORD tmsOn, WORD tmsOff, BOOL fRight);
void    LedStrobeOpposite(WORD dtcR, WORD dtcG, WORD dtcB, WORD tmsOn, WORD tmsOff, BOOL fInvert);

void    LedCalibrationStart();
void    LedCalibrationSuccess();
void    LedCalibrationFailed();
void    LedDispBrakeCoast();

void    LedRestoreCalStart();
void    LedRestoreCalComplete();

BOOL    LedInitComplete();

/* ------------------------------------------------------------ */


#endif
