/************************************************************************/
/*									*/
/*  Led.c - Motor Controller LED Functions                              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to    */
/*  configure and control the Motor Controller H-Bridge.                */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/15/2016 (MichaelA): created			                */
/*  04/21/2016 (MichaelA): added additional functions for configuring   */
/*      the LEDs to display different patterns and cleaned up code      */
/*  05/02/2016 (MichaelA): fixed a bug in LedTick that resulted in the  */
/*      circular blink rate not being updated when it should be         */
/*  06/27/2016 (MichaelA): added code to conditionally compile in       */
/*      support for a brake/cal LED. Please note that unless            */
/*      "BRAKE_CAL_LED" is defined the LED will not be driven           */
/*  07/06/2016 (MichaelA): fixed bug that prevented brake/coast state   */
/*      from being displayed while no input signal is present           */
/*  07/06/2016 (MichaelA): modified LedInit() such that the brake/cal   */
/*      LED (if present) is driven to an appropriate state during       */
/*      initialization                                                  */
/*  08/02/2016 (MichaelA): fixed bug that prevented the fInitComplete   */
/*      LED flag from being set when the device powered on in a fault   */
/*      state                                                           */
/*  09/01/2016 (MichaelA): modified LetTick() such that an overcurrent  */
/*      error no performs an opposite blink between red and orange      */
/*      instead of red and red                                          */
/*  09/01/2016 (MichaelA): modified LedTick() such that the no link     */
/*      state now performs an oppposite blink between red and red       */
/*      instead of orange and orange                                    */
/*  09/07/2016 (MichaelA): added new LED state and modified LedTick()   */
/*      such that it checks to see if the controller is halted and if   */
/*      so goes to the Halted state instead of the Neutral state. This  */
/*      change was made so that the presence of a halted CAN bus can be */
/*      displayed to the user                                           */
/*  11/02/2016 (MichaelA): added LedRestoreCalStart() and               */
/*      LedRestoreCalComplete() for setting flags that allow the LEDs   */
/*      to inform the user when the servo calibration constants have    */
/*      been restored. Note:  LedTick() was also modified               */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <string.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Ctrlr.h"
#include "HBridge.h"
#include "Led.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define the states that the LED state machine can be in.
*/
#define ledstInit           0
#define ledstInitDelay      1
#define ledstInitComplete   2
#define ledstFwdMax         3
#define ledstFwd            4
#define ledstNeutral        5
#define ledstRev            6
#define ledstRevMax         7
#define ledstLimitFault     8
#define ledstNoLink         9
#define ledstFault          10
#define ledstDelay          11
#define ledstCalibrate      12
#define ledstRestoreCal     13
#define ledstDispBC1        14
#define ledstDispBC2        15
#define ledstHalted         16

/* Define the maximum number of LED on/off states.
*/
#define cledoostMax     4

/* Define the LED duty cycle for displaying status in various states.
*/
#if defined(DEAD)
#define dtcLedInit      64
#define dtcLedCal       64
#define dtcLedFwdMax    64
#define dtcLedFwd       64
#define dtcLedNeutral   64
#define dtcLedRev       64
#define dtcLedRevMax    64
#define dtcLedNoLink    64
#define dtcLedFault     64
#define dtcLedDispBC    64
#else
#define dtcLedInit      256
#define dtcLedCal       256
#define dtcLedFwdMax    256
#define dtcLedFwd       256
#define dtcLedNeutral   256
#define dtcLedRev       256
#define dtcLedRevMax    256
#define dtcLedNoLink    256
#define dtcLedFault     256
#define dtcLedDispBC    256
#define dtcLedHalted    256
#endif

/* Define macros that can be used to disable and enable interrupts.
*/
#define DISABLE_INTERRUPTS \
        INTCON2bits.GIE = 0;

#define ENABLE_INTERRUPTS \
        INTCON2bits.GIE = 1;

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* Declare variable used to keep track of the current LED state for the LED
** state machine
*/
static  BYTE        ledstCur;   // current state of the LED state machine
static  LEDFLAGS    ledflgs;    // current LED state flags
static  WORD        ctickDelay; // number of ticks remaining in the LED delay state

/* Declare variables used to keep track of the current LED on/off state for the
** LED on/off state machine.
*/
static  LEDOOST rgledoost[cledoostMax];
static  BYTE    iledoost; // current index into LED on/off state array
static  WORD    ctickSt;  // number of ticks remaining for the current on/off state
static  WORD    ctickDtc; // number of ticks remaining for the current duty cycle

/* Define the duty cycles that are applied to the Red, Green, and Blue LEDs
** to implement a glowing affect.
*/
static const WORD   rgdtcGlow[] = {
    0,
    1,
    2,
    4,
    6,
    8,
    10,
    12,
    16,
    20,
    24, // maybe take this out but it seems ok
    32,
    48,
    64,
    80,
    96,
    128,
    160,
    192,
    224,
    256
};

#define cdtcGlowMax sizeof(rgdtcGlow) / sizeof(WORD)

static  BYTE    idtcGlow;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	LedTick
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function ...
*/
void
LedTick() {

    static int16_t  vltgLast = 0;
    static BOOL     fBC = fFalse;
    static BOOL     fReduceDTCLast = fFalse;
    int16_t         vltgCur;
    WORD            tmsUpdate;
    WORD            fsFlts;
    BOOL            fReduceDTC;

    /* Get a copy of the controller fault flags. Please note that the fault
    ** flags are set and cleared in ISR context. It may or may not be necessary
    ** to disable interrupts while the flags are being read. Look at the
    ** disassembly to see how many instructions are used to perform the set,
    ** clear, and read operations to determine whether or not it's necessary.
    */
    DISABLE_INTERRUPTS
    fsFlts = CtrlrGetFaults();
    ENABLE_INTERRUPTS

    /* Check to see if a calibration proceedure was started or we need to
    ** display the current Brake/Coast setting.
    */
    DISABLE_INTERRUPTS
    if ( ledflgs.fCalStart ) {
        /* Calibration was started. Clear the flag so that we don't process it
        ** again and configure the LEDs to display the current status.
        */
        ledflgs.fCalStart = 0;
        ENABLE_INTERRUPTS
        LedAllBlink(colorBlue, dtcLedCal, 250);
        ledstCur = ledstCalibrate;
    }
    else if ( ledflgs.fDispBC ) {
        fBC = HBridgeGetBrakeCoast();
        ledflgs.fDispBC = 0;
        ENABLE_INTERRUPTS
        
#if defined(BRAKE_CAL_LED)
        if ( fBC ) {
            latLedBC |= (1 << bnLedBC);
        }
        else {
            latLedBC &= ~(1 << bnLedBC);
        }
#else
        LedAllBlink(colorYellow, dtcLedDispBC, 250);
        ctickDelay = LedMsToTicks(1000);
        ledstCur = ledstDispBC1;
#endif
    }
    else if ( ledflgs.fRestoreCalStart ) {
        ledflgs.fRestoreCalStart = 0;
        ledflgs.fInitComplete = fTrue; // skip progressive blue glow on
        ENABLE_INTERRUPTS
        LedOppositeBlink(colorGreen, colorGreen, dtcLedCal, 100, 0);
        ledstCur = ledstRestoreCal;
    }
    else {
        ENABLE_INTERRUPTS
    }

    /* Check to see if a fault occured. If a fault did occur we only want to
    ** change status if we aren't already displaying some other important status
    ** information.
    */
    if (( 0 != fsFlts ) &&
        ( ledstNoLink != ledstCur ) &&
        ( ledstFault != ledstCur ) &&
        ( ledstDelay != ledstCur ) &&
        ( ledstInitDelay != ledstCur )) {

        /* Determine which color to set based on the fault that has occured.
        */
        if ( fltOverCurrent == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorOrange, dtcLedFault, 500, 250);
        }
        else if ( fltOverTemp == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorGreen, dtcLedFault, 500, 250);
        }
        else if ( (fltOverCurrent | fltOverTemp) == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorYellow, dtcLedFault, 500, 250);
        }
        else if ( fltUnderVoltage == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorBlue, dtcLedFault, 500, 250);
        }
        else if ( (fltOverCurrent | fltUnderVoltage) == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorFuchsia, dtcLedFault, 500, 250);
        }
        else if ( (fltOverTemp | fltUnderVoltage) == (fsFlts & 0x07 ) ) {
            LedOppositeBlink(colorRed, colorCyan, dtcLedFault, 500, 250);
        }
        else {
            LedOppositeBlink(colorRed, colorWhite, dtcLedFault, 500, 250);
        }

        ledstCur = ledstFault;
    }

    /* Perform an appropriate action based on the current state of the LED state
    ** machine.
    */
    switch ( ledstCur ) {
        case ledstInit:
            LedProgressiveGlow(colorBlue, 100, 4000, 1000, fFalse);
            ctickDelay = LedMsToTicks(5000);
            ledstCur = ledstInitDelay;
            break;

        case ledstInitDelay:
            ctickDelay--;
            if ( 0 == ctickDelay ) {
                LedAllSolid(colorBlack, dtcLedInit);
                ledstCur = ledstInitComplete;
                ledflgs.fInitComplete = fTrue;
            }
            break;

        case ledstInitComplete:
        case ledstFwdMax:
        case ledstFwd:
        case ledstNeutral:
        case ledstRev:
        case ledstRevMax:
        case ledstLimitFault:
            /* Check to see if the controller is halted, and if so, advance to
            ** the halted state.
            */
            if ( CtrlrIsHalted() ) {
                LedOppositeBlink(colorOrange, colorOrange, dtcLedHalted, 500, 250);
                ledstCur = ledstHalted;
                break;
            }

            /* Get the current voltage being applied to the H-Bridge.
            */
            vltgCur = CtrlrGetVoltage();

            /* M00TODO: add code to check for a non-zero voltage being applied
            ** while there is a forward or reverse limit fault that's in the
            ** same direction as the applied voltage.
            */
            if (( 32767 == vltgCur ) && ( ledstFwdMax != ledstCur )) {
                /* The maximum forward voltage is applied and we aren't
                ** currently in the FwdMax state. Configure the LEDs to display
                ** solid green and advance to the FwdMax state.
                */
                LedAllSolid(colorGreen, dtcLedFwdMax);
                ledstCur = ledstFwdMax;
            }
            else if (( 0 < vltgCur ) && ( 32767 > vltgCur )) {
                /* A positive voltage is applied to the H-Bridge. Configure the
                ** LEDs to display a green circular blink in the
                ** counter-clockwise direction, which should indicate that the
                ** motor controller is moving forward. The blink rate should be
                ** proportional to the applied voltage.
                */
                tmsUpdate = 500 - (((uint32_t)vltgCur * 500) / 32766);
                if ( 75 > tmsUpdate ) {
                    tmsUpdate = 75;
                }

                /* Determine whether or not the motor controller is operating
                ** at a reduce duty cycle due to the current ambient temperature
                ** inside of the case. Adjust the LED color as necessary.
                */
                fReduceDTC = CtrlrIsDTCReduced();
                if ( ledstFwd != ledstCur ) {
                    LedCircularBlink(fReduceDTC ? colorCyan : colorGreen, dtcLedFwd, tmsUpdate, fFalse);
                    fReduceDTCLast = fReduceDTC;
                }
                else if ( vltgCur != vltgLast ) {
                    LedCircularBlinkAdjustRate(tmsUpdate);
                }

                if ( fReduceDTCLast != fReduceDTC ) {
                    LedCircularBlinkAdjustColor(fReduceDTC ? colorCyan : colorGreen, dtcLedFwd);
                    fReduceDTCLast = fReduceDTC;
                }
                
                ledstCur = ledstFwd;
                vltgLast = vltgCur;
            }
            else if (( 0 == vltgCur ) && ( ledstNeutral != ledstCur )) {
                /* The neutral throttle is applied to the H-Bridge. Configure
                ** the LEDs to display a solid orange and advance to the Neutral
                ** state.
                */
                LedAllSolid(colorOrange, dtcLedNeutral);
                ledstCur = ledstNeutral;
            }
            else if (( -32768 < vltgCur ) && ( 0 > vltgCur)) {
                /* A negative voltage is applied to the H-Bridge. Configure the
                ** LEDs to display a red circular blink in the clockwise
                ** direction, which should indicate that the motor controller is
                ** moving backward. The blink rate should be proportional to the
                ** applied voltage.
                */
                tmsUpdate = (((uint32_t)(32767 + vltgCur) * 500) / 32766);
                if ( 75 > tmsUpdate ) {
                    tmsUpdate = 75;
                }

                /* Determine whether or not the motor controller is operating
                ** at a reduce duty cycle due to the current ambient temperature
                ** inside of the case. Adjust the LED color as necessary.
                */
                fReduceDTC = CtrlrIsDTCReduced();
                if ( ledstRev != ledstCur ) {
                    LedCircularBlink(fReduceDTC ? colorFuchsia : colorRed, dtcLedRev, tmsUpdate, fTrue);
                    fReduceDTCLast = fReduceDTC;
                }
                else if ( vltgCur != vltgLast ) {
                    LedCircularBlinkAdjustRate(tmsUpdate);
                }
                
                if ( fReduceDTCLast != fReduceDTC ) {
                    LedCircularBlinkAdjustColor(fReduceDTC ? colorFuchsia : colorRed, dtcLedRev);
                    fReduceDTCLast = fReduceDTC;
                }

                ledstCur = ledstRev;
                vltgLast = vltgCur;
            }
            else if (( -32768 == vltgCur ) && ( ledstRevMax != ledstCur )) {
                 /* The maximum reverse voltage is applied and we aren't
                ** currently in the RevMax state. Configure the LEDs to display
                ** solid red and advance to the RevMax state.
                */
                LedAllSolid(colorRed, dtcLedRevMax);
                ledstCur = ledstRevMax;
            }

            /* We've now handled all of the above cases so break out of the
            ** switch statement.
            */
            break;

        case ledstNoLink:
            /* We previously lost the SERVO/CAN link or there was never a link
            ** to begin with. Check to see if we have a link now.
            */
            if ( linktNone != CtrlrGetLinkType() ) {
                /* A link has been established. If the controller is halted then
                ** assume that we have a CAN link and set the LEDs to opposite
                ** blink and proceed to the Halted state. Otherwise, configure
                ** Configure the LEDs to display a solid orange and advance to
                **  the Neutral state.
                */
                if ( CtrlrIsHalted() ) {
                    LedOppositeBlink(colorOrange, colorOrange, dtcLedHalted, 500, 250);
                    ledstCur = ledstHalted;
                }
                else {
                    LedAllSolid(colorOrange, dtcLedNeutral);
                    ledstCur = ledstNeutral;
                }
            }
            break;

        case ledstFault:
            /* A fault previously occured. Check to see if the fault flags have
            ** all been cleared, and if so, advance to the halted or neutral
            ** state.
            */
            if ( 0 == fsFlts ) {
                if ( CtrlrIsHalted() ) {
                    LedOppositeBlink(colorOrange, colorOrange, dtcLedHalted, 500, 250);
                    ledstCur = ledstHalted;
                }
                else {
                    LedAllSolid(colorOrange, dtcLedNeutral);
                    ledstCur = ledstNeutral;
                }
            }
            break;

        case ledstDelay:
            /* Decrement the delay count. If it reaches zero then we should
            ** turn on the orange LED and advance to the halted or neutral
            ** state as appropriate.
            */
            ctickDelay--;
            if ( 0 == ctickDelay ) {
                if ( CtrlrIsHalted() ) {
                    LedOppositeBlink(colorOrange, colorOrange, dtcLedHalted, 500, 250);
                    ledstCur = ledstHalted;
                }
                else {
                    LedAllSolid(colorOrange, dtcLedNeutral);
                    ledstCur = ledstNeutral;
                }
            }
            break;

        case ledstCalibrate:
            /* Check to see if calibration has successfully completed or if it
            ** has failed.
            */
            DISABLE_INTERRUPTS
            if ( ledflgs.fCalSuccess ) {
                ledflgs.fCalSuccess = 0;
                ENABLE_INTERRUPTS
                LedAllBlink(colorGreen, dtcLedCal, 250);
                ctickDelay = LedMsToTicks(1000);
                ledstCur = ledstDelay;
            }
            else if ( ledflgs.fCalFailed ) {
                ledflgs.fCalFailed = 0;
                ENABLE_INTERRUPTS
                LedAllBlink(colorRed, dtcLedCal, 250);
                ctickDelay = LedMsToTicks(1000);
                ledstCur = ledstDelay;
            }
            else {
                ENABLE_INTERRUPTS
            }
            break;

        case ledstRestoreCal:
            DISABLE_INTERRUPTS
            if ( ledflgs.fRestoreCalComplete ) {
                ledflgs.fRestoreCalComplete = 0;
                ENABLE_INTERRUPTS
                ctickDelay = LedMsToTicks(500);
                ledstCur = ledstDelay;
            }
            else {
                ENABLE_INTERRUPTS
            }
            break;

        case ledstDispBC1:
            ctickDelay--;
            if ( 0 == ctickDelay ) {
                if ( fBC ) {
                    LedAllSolid(colorRed, dtcLedDispBC);
                }
                else {
                    LedAllSolid(colorGreen, dtcLedDispBC);
                }
                ctickDelay = LedMsToTicks(1000);
                ledstCur = ledstDispBC2;
            }
            break;

        case ledstDispBC2:
            ctickDelay--;
            if ( 0 == ctickDelay ) {
                LedAllBlink(colorYellow, dtcLedDispBC, 250);
                ctickDelay = LedMsToTicks(1000);
                ledstCur = ledstDelay;
            }
            break;

        case ledstHalted:
            if ( ! CtrlrIsHalted() ) {
                LedAllSolid(colorOrange, dtcLedNeutral);
                ledstCur = ledstNeutral;
            }
            break;
    }

    /* Check to see if we've lost the SERVO or CAN link.
    */
    if (( ledflgs.fInitComplete ) &&
        ( linktNone == CtrlrGetLinkType() ) &&
        ( ledstNoLink != ledstCur ) &&
        ( ledstDispBC1 != ledstCur ) &&
        ( ledstDispBC2 != ledstCur ) &&
        ( ledstRestoreCal != ledstCur ) &&
        ( ledstDelay != ledstCur )) {
        /* We've lost the input signal. Blink top and bottom LEDs orange to
        ** indicate the loss of signal.
        */
        LedOppositeBlink(colorRed, colorRed, dtcLedNoLink, 500, 250);
        ledstCur = ledstNoLink;
    }

    /* Check to see if we need to perform a LED on/off state transition.
    */
    if ( 0 == ctickSt ) {
        /* We've reached the tick limit for the current state. Transition to the
        ** next state.
        */
        iledoost++;
        if ( cledoostMax <= iledoost ) {
            iledoost = 0;
        }

        /* Progressive glow supports a glow up and down option. When glow down
        ** is enabled the fGlowR, fGlowG, or fGlowB flags will be set in the
        ** fsFlags variable associated with on/off state 1. In this case we
        ** don't want to update the physical state of the LEDs at this time nor
        ** do we want to modify the duty cycle from what it was during its
        ** previous state. However, the the tick values should be updated.
        */
        if (( 1 != iledoost ) ||
            (( 0 == rgledoost[iledoost].fGlowR) &&
             ( 0 == rgledoost[iledoost].fGlowG) &&
             ( 0 == rgledoost[iledoost].fGlowB))) {

            /* Set the physical state of the LEDs to correspond to the state
            ** specified for this on/off state.
            */
            LedSet(&(rgledoost[iledoost].leds));

            /* Rest our index into the array of duty cycles that are used to
            ** implement a progressive glow.
            */
            idtcGlow = 0;
        }

        if (( 1 == iledoost ) &&
            (( rgledoost[iledoost].fGlowR ) ||
             ( rgledoost[iledoost].fGlowG ) ||
             ( rgledoost[iledoost].fGlowB )) &&
            ( 0 < idtcGlow )) {
            idtcGlow--;
        }

        /* Set the temporary state variables that correspond to the current
        ** on off state.
        */
        ctickSt = rgledoost[iledoost].ctickSt;
        ctickDtc = rgledoost[iledoost].ctickDtc;

    }
    else {
        /* The tick limit for the current state hasn't been reached. Decrement
        ** the tick count.
        */
        ctickSt--;

        /* If we are supposed to toggle the duty cycle while in this state then
        ** decrement the tick count and update the duty cycle as necessary.
        */
        if ( 0 < rgledoost[iledoost].ctickDtc ) {
            ctickDtc--;
            if ( 0 == ctickDtc ) {
                /* Adjust the Red LEDs duty cycle as required for this state.
                */
                if ( rgledoost[iledoost].fGlowR ) {
                    if ( rgledoost[iledoost].fHalfDtcR ) {
                        SetRedDTC(rgdtcGlow[idtcGlow] >> 1);
                    }
                    else {
                        SetRedDTC(rgdtcGlow[idtcGlow]);
                    }
                }

                /* Adjust the Green LEDs duty cycle as required for this state.
                */
                if ( rgledoost[iledoost].fGlowG ) {
                    if ( rgledoost[iledoost].fHalfDtcG ) {
                        SetGreenDTC(rgdtcGlow[idtcGlow] >> 1);
                    }
                    else {
                        SetGreenDTC(rgdtcGlow[idtcGlow]);
                    }
                }

                /* Adjust the Blue LEDs duty cycle as required for this state.
                */
                if ( rgledoost[iledoost].fGlowB ) {
                    if ( rgledoost[iledoost].fHalfDtcB ) {
                        SetBlueDTC(rgdtcGlow[idtcGlow] >> 1);
                    }
                    else {
                        SetBlueDTC(rgdtcGlow[idtcGlow]);
                    }
                }

                /* Update our index into the array that defines the duty cycle
                ** values applied to implement a progressive glow. If we are
                ** in led on/off state 1 then we are supposed to reduce the duty
                ** cycle, which means going backwards in the array.
                */
                if ( 1 != iledoost ) {
                    if ( idtcGlow < (cdtcGlowMax - 1) ) {
                        idtcGlow++;
                    }
                }
                else {
                    if ( 0 < idtcGlow ) {
                        idtcGlow--;
                    }
                }

                /* Reload the duty cycle for this state into the tick counter.
                */
                ctickDtc = rgledoost[iledoost].ctickDtc;
            }
        }
    }
}

/* ------------------------------------------------------------ */
/***	LedInit
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function initializes the onboard LEDs. It configures the individual
**      enable pins as outputs and drives them low, disables Timer 5 and the
**      output compare modules, then configures the output compare modules and
**      Timer 5 such that OC1, OC2, and OC3 are used to PWM the Red, Green, and
**      Blue PWM signals with a refresh rate of approximately 122 Hz with 8-bit
**      resolution. All LED enable signals are driven to their off state and the
**      Red, Green, and Blue duty cycles are set to a default value of 0.
*/
void
LedInit() {

    /* Disable the output compare modules and their time base (Timer 5).
    */
    OC1CON1 = 0;
    OC2CON1 = 0;
    OC3CON1 = 0;
    T5CONbits.TON = 0;

    /* Initialize the LEDs. Please note that the LEDs are active low and that
    ** some of them are powered by the 5V rail. In order to ensure that the LEDs
    ** remain off they must be tri-stated. We will turn them on by disabling the
    ** the tri-state buffers, at which point they should be driven low.
    */
    trsLedR1 |= (1 << bnLedR1);
    trsLedR2 |= (1 << bnLedR2);
    trsLedR3 |= (1 << bnLedR3);
    trsLedR4 |= (1 << bnLedR4);
    trsLedG1 |= (1 << bnLedG1);
    trsLedG2 |= (1 << bnLedG2);
    trsLedG3 |= (1 << bnLedG3);
    trsLedG4 |= (1 << bnLedG4);
    trsLedB1 |= (1 << bnLedB1);
    trsLedB2 |= (1 << bnLedB2);
    trsLedB3 |= (1 << bnLedB3);
    trsLedB4 |= (1 << bnLedB4);

    latLedR1 &= ~(1 << bnLedR1);
    latLedR2 &= ~(1 << bnLedR2);
    latLedR3 &= ~(1 << bnLedR3);
    latLedR4 &= ~(1 << bnLedR4);
    latLedG1 &= ~(1 << bnLedG1);
    latLedG2 &= ~(1 << bnLedG2);
    latLedG3 &= ~(1 << bnLedG3);
    latLedG4 &= ~(1 << bnLedG4);
    latLedB1 &= ~(1 << bnLedB1);
    latLedB2 &= ~(1 << bnLedB2);
    latLedB3 &= ~(1 << bnLedB3);
    latLedB4 &= ~(1 << bnLedB4);

    /* Configure the LED PWM signals as outputs and drive them to the state
    ** that will disable all LEDs of a given color.
    */
    latPwmR |= (1 << bnPwmR);
    trsPwmR &= ~(1 << bnPwmR);
    latPwmG &= ~(1 << bnPwmG);
    trsPwmG &= ~(1 << bnPwmG);
    latPwmB &= ~(1 << bnPwmB);
    trsPwmB &= ~(1 << bnPwmB);

#if defined(BRAKE_CAL_LED)
    /* Configure Brake/CAL LED as an output and drive it to the state
    ** corresponding to the current Brake/Coast setting.
    */
    trsLedBC &= ~(1 << bnLedBC);
    if ( HBridgeGetBrakeCoast() ) {
        latLedBC |= (1 << bnLedBC);
    }
    else {
        latLedBC &= ~(1 << bnLedBC);
    }
#endif

    /* Initialize the variables used to keep track of the current LED state
    ** machine state.
    */
    ledstCur = ledstInit;
    ledflgs.fsFlags = 0;
    ctickDelay = 0;

    /* Initialize the LED on/off state variables.
    */
    memset(rgledoost, 0, sizeof(rgledoost));
    iledoost = cledoostMax - 1; // this will put us in the correct initial state
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;

    /* Configure Output Compare 1. This output compare module will be used to
    ** PWM the Red LED Enable signal.
    */
    OC1CON2 = 0;
    OC1CON1bits.OCSIDL = 0; // OC module continues to operate during CPU idle mode
    OC1CON1bits.OCTSEL = 0b011; // use timer 5 clock as clock source
    OC1CON1bits.ENFLT1 = 0; // disable Fault 1/B input
    OC1CON1bits.ENFLT0 = 0; // disable Fault 0/A input
    OC1CON1bits.TRIGMODE = 0; // TRIGSTAT bit is cleared by software
    OC1CON2bits.OCINV = 1; // invert output
    OC1R = 0; // set initial duty cycle to 0
    OC1RS = 8192; // set period for 122.01 Hz
    OC1CON2bits.SYNCSEL = 0x1F; // use OCxRS for synchronization (set period)
    OC1CON1bits.OCM = 0b110; // start edge aligned PWM mode

    /* Configure Output Compare 2. This output compare module will be used to
    ** PWM the Green LED Enable signal.
    */
    OC2CON2 = 0;
    OC2CON1bits.OCSIDL = 0; // OC module continues to operate during CPU idle mode
    OC2CON1bits.OCTSEL = 0b011; // use timer 5 clock as clock source
    OC2CON1bits.ENFLT1 = 0; // disable Fault 1/B input
    OC2CON1bits.ENFLT0 = 0; // disable Fault 0/A input
    OC2CON1bits.TRIGMODE = 0; // TRIGSTAT bit is cleared by software
    OC2R = 0; // set initial duty cycle to 0
    OC2RS = 8192; // set period for 122.01 Hz
    OC2CON2bits.SYNCSEL = 0x1F; // use OCxRS for synchronization (set period)
    OC2CON1bits.OCM = 0b110; // start edge aligned PWM mode

    /* Configure Output Compare 3. This output compare module will be used to
    ** PWM the Blue LED enable signal.
    */
    OC3CON2 = 0;
    OC3CON1bits.OCSIDL = 0; // OC module continues to operate during CPU idle mode
    OC3CON1bits.OCTSEL = 0b011; // use timer 5 clock as clock source
    OC3CON1bits.ENFLT1 = 0; // disable Fault 1/B input
    OC3CON1bits.ENFLT0 = 0; // disable Fault 0/A input
    OC3CON1bits.TRIGMODE = 0; // TRIGSTAT bit is cleared by software
    OC3R = 0; // set initial duty cycle to 0
    OC3RS = 8192; // set period for 122.01 Hz
    OC3CON2bits.SYNCSEL = 0x1F; // use OCxRS for synchronization (set period)
    OC3CON1bits.OCM = 0b110; // start edge aligned PWM mode

    /* Configure Timer 5. This timer will be used as the time base for the
    ** output compare modules, which are used to generate the LED PWM output. It
    ** will also be used to make calls to the LedTick() functions at an
    ** interval of 500us. The calls to LedTick() are made by polling the
    ** interrupt flag in the main loop, which removes the need to set a flag in
    ** an interrupt service routine and then later clear it in mainline code.
    ** Note: Do NOT clear or write the timer count register after the timer has
    ** been enabled. Doing so will cause a sync signal to be output to the
    ** output compare modules and cause glitches on the LED PWM output.
    */
    T5CONbits.TCS = 0; // use internal peripheral bus clock
    T5CONbits.TGATE = 0; // disable gated timer mode
    T5CONbits.TCKPS = 0b10; // 1:64 prescaler
    TMR5 = 0; // clear timer count
    PR5 = 499; // period match of 500us
    IFS1bits.T5IF = 0; // clear interrupt flag
    IEC1bits.T5IE = 0; // disable interrupt

    /* Map PPS pins for output compare modules.
    */
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    ppsPwmR = 0b010000; // output compare 1
    ppsPwmG = 0b010001; // output compare 2
    ppsPwmB = 0b010010; // output compare 3
    __builtin_write_OSCCONL(OSCCON | (1<<6));

    /* Start Timer 5, which generates the timebase for the output compare
    ** modules.
    */
    T5CONbits.TON = 1;
}

/* ------------------------------------------------------------ */
/***	LedSet
**
**  Parameters:
**      pledsSet    - pointer to variable containing physical state to set for
**                    the LEDs
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the physical state of the onboard LEDs to the state
**      specified in the variable pointed to by pledsSet. The on and off state
**      is set for all LEDs and the duty cycle for each color (Red, Green, Blue)
**      is set to the duty cycle specified.
*/
void
LedSet(LEDS* pledsSet) {

    SetRedDTC(pledsSet->dtcRed);
    SetGreenDTC(pledsSet->dtcGreen);
    SetBlueDTC(pledsSet->dtcBlue);

    if ( NULL == pledsSet) {
        return;
    }

    if ( pledsSet->R1 ) {
        LED_R1_ON
    }
    else {
        LED_R1_OFF
    }

    if ( pledsSet->R2 ) {
        LED_R2_ON
    }
    else {
        LED_R2_OFF
    }

    if ( pledsSet->R3 ) {
        LED_R3_ON
    }
    else {
        LED_R3_OFF
    }

    if ( pledsSet->R4 ) {
        LED_R4_ON
    }
    else {
        LED_R4_OFF
    }

    if ( pledsSet->G1 ) {
        LED_G1_ON
    }
    else {
        LED_G1_OFF
    }

    if ( pledsSet->G2 ) {
        LED_G2_ON
    }
    else {
        LED_G2_OFF
    }

    if ( pledsSet->G3 ) {
        LED_G3_ON
    }
    else {
        LED_G3_OFF
    }

    if ( pledsSet->G4 ) {
        LED_G4_ON
    }
    else {
        LED_G4_OFF
    }

    if ( pledsSet->B1 ) {
        LED_B1_ON
    }
    else {
        LED_B1_OFF
    }

    if ( pledsSet->B2 ) {
        LED_B2_ON
    }
    else {
        LED_B2_OFF
    }

    if ( pledsSet->B3 ) {
        LED_B3_ON
    }
    else {
        LED_B3_OFF
    }

    if ( pledsSet->B4 ) {
        LED_B4_ON
    }
    else {
        LED_B4_OFF
    }
}

/* ------------------------------------------------------------ */
/***	LedGet
**
**  Parameters:
**      pledsGet    - pointer to variable to receive current physical LED state
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine reads the state of the LED pins and the duty cycle
**      registers and stores them in the variable poitned to by pledsGet.
*/
void
LedGet(LEDS* pledsGet) {

    if ( NULL == pledsGet ) {
        return;
    }

    pledsGet->R1 = ( latLedR1 & (1 << bnLedR1) ) ? 1 : 0;
    pledsGet->R2 = ( latLedR2 & (1 << bnLedR2) ) ? 1 : 0;
    pledsGet->R3 = ( latLedR3 & (1 << bnLedR3) ) ? 1 : 0;
    pledsGet->R4 = ( latLedR4 & (1 << bnLedR4) ) ? 1 : 0;

    pledsGet->G1 = ( latLedG1 & (1 << bnLedG1) ) ? 1 : 0;
    pledsGet->G2 = ( latLedG2 & (1 << bnLedG2) ) ? 1 : 0;
    pledsGet->G3 = ( latLedG3 & (1 << bnLedG3) ) ? 1 : 0;
    pledsGet->G4 = ( latLedG4 & (1 << bnLedG4) ) ? 1 : 0;

    pledsGet->B1 = ( latLedB1 & (1 << bnLedB1) ) ? 1 : 0;
    pledsGet->B2 = ( latLedB2 & (1 << bnLedB2) ) ? 1 : 0;
    pledsGet->B3 = ( latLedB3 & (1 << bnLedB3) ) ? 1 : 0;
    pledsGet->B4 = ( latLedB4 & (1 << bnLedB4) ) ? 1 : 0;

    pledsGet->dtcRed = GetRedDTC();
    pledsGet->dtcGreen = GetGreenDTC();
    pledsGet->dtcBlue = GetBlueDTC();
}

/* ------------------------------------------------------------ */
/***	LedAllBlink
**
**  Parameters:
**      colorDisp   - color displayed by the LEDs
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**      tmsOn       - number of milliseconds and LED is on before a rotation
**                    oocurs
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the all
**      four of the LEDs display the specified color for the specified duration.
**      The LEDs will display the specified color for tmsOn milliseconds and
**      then will turn off. They will remain off for tmsOn milliseconds, after
**      which they will turn back on. The process will repeat indefinitely.
*/
void    
LedAllBlink(BYTE colorDisp, WORD dtcMaster, WORD tmsOn) {

    BYTE    i;
    WORD    ctick;
    
    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0x0FFF;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = 0;
        rgledoost[i].fsFlags = 0;
    }

    /* The LEDs will be on during state 0 and off during all other states.
    ** Set the duty cycle for state 0 based on the color that the user wants to
    ** display.
    */
    switch ( colorDisp ) {

        case colorRed:
            rgledoost[0].leds.dtcRed = dtcMaster;
            break;

        case colorGreen:
            rgledoost[0].leds.dtcGreen = dtcMaster;
            break;

        case colorBlue:
            rgledoost[0].leds.dtcBlue = dtcMaster;
            break;

        case colorYellow:
            rgledoost[0].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = dtcMaster;
            break;

        case colorFuchsia:
            rgledoost[0].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcBlue = dtcMaster;
            break;

        case colorCyan:
            rgledoost[0].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = dtcMaster;
            break;

        case colorOrange:
            rgledoost[0].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = (dtcMaster >> 1);
            break;

        case colorWhite:
            rgledoost[0].leds.dtcRed = dtcMaster;;
            rgledoost[0].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = dtcMaster;
            break;

        case colorBlack:
        default:
            break;
    }

    /* State 0 is used to implement the LED on time while state 1 implements the
    ** off time. For this LED pattern state 2 and 3 serve as dummy states.
    */
    ctick = LedMsToTicks(tmsOn);
    rgledoost[0].ctickSt = ctick;
    rgledoost[1].ctickSt = ctick;

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedAllSolid
**
**  Parameters:
**      colorDisp   - color displayed by the LEDs
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the all
**      four of the LEDs will turn on and display the specified color
**      indefinitely.
*/
void
LedAllSolid(BYTE colorDisp, WORD dtcMaster) {

    BYTE i;

    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0x0FFF;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = 0xFFFF;
        rgledoost[i].fsFlags = 0;
    }

    /* Set the duty cycle for the Red, Green, and Blue LEDs appropriately
    ** to display the specified color.
    */
    switch ( colorDisp ) {

        case colorRed:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            break;

        case colorGreen:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorBlue:
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorYellow:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorFuchsia:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorCyan:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorOrange:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = (dtcMaster >> 1);
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = (dtcMaster >> 1);
            break;

        case colorWhite:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorBlack:
        default:
            break;
    }

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedAlternateBlink
**
**  Parameters:
**      colorTop    - color to set for the top LEDs (LD1 and LD2)
**      colorBottom - color to set for the bottom LEDs (LD3 and LD4)
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**      tmsOn       - number of milliseconds LEDs are turned on during top or
**                    bottom state
**      tmsDelay    - number of milliseconds LEDs are turned off between top and
**                    bottom state
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the top
**      LEDs (LD1 and LD2) display color specified by colorTop while the bottom
**      LEDs (LD3 and LD4) display the color specified by colorBottom. The LEDs
**      remain in this initial state for tmsOn milliseconds. The top and bottom
**      colors are then swapped and the LEDs remain in this alternate state for
**      tmsOn milliseconds. If a non-zero delay is specified for tmsDelay then
**      the LEDs will turn off for tmsDelay milliseconds between the top and
**      bottom LEDs swapping colors.
**
**  Notes:
**      This function does NOT support colorOrange. Passing in a value of
**      colorOrange for colorTop or colorBottom will result in the LEDs
**      remaining off during that portion of the LED on time.
**
**      This function may be used to make a color alternate between the top and
**      bottom LEDs. This can be done by specifying the desired color for the
**      colorTop parameter and specifying colorBlack for the colorBottom
**      parameter.
*/
void
LedAlternateBlink(BYTE colorTop, BYTE colorBottom, WORD dtcMaster, WORD tmsOn, WORD tmsDelay) {

    BYTE    i;
    
    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0;
        rgledoost[i].leds.dtcRed = dtcMaster;
        rgledoost[i].leds.dtcGreen = dtcMaster;
        rgledoost[i].leds.dtcBlue = dtcMaster;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].fsFlags = 0;
    }

    /* Check to see if the Red LEDs should be on for the top color.
    */
    if (( colorRed == colorTop ) ||
        ( colorYellow == colorTop ) ||
        ( colorFuchsia == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.R1 = rgledoost[0].leds.R2 = 1;
        rgledoost[2].leds.R3 = rgledoost[2].leds.R4 = 1;
    }

    /* Check to see if the Green LEDs should be on for the top color.
    */
    if (( colorGreen == colorTop ) ||
        ( colorYellow == colorTop ) ||
        ( colorCyan == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.G1 = rgledoost[0].leds.G2 = 1;
        rgledoost[2].leds.G3 = rgledoost[2].leds.G4 = 1;
    }

    /* Check to see if the Blue LEDs should be on for the top color.
    */
    if (( colorBlue == colorTop ) ||
        ( colorFuchsia == colorTop ) ||
        ( colorCyan == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.B1 = rgledoost[0].leds.B2 = 1;
        rgledoost[2].leds.B3 = rgledoost[2].leds.B4 = 1;
    }
    
    /* Check to see if the Red LEDs should be on for the bottom color.
    */
    if (( colorRed == colorBottom ) ||
        ( colorYellow == colorBottom ) ||
        ( colorFuchsia == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[0].leds.R3 = rgledoost[0].leds.R4 = 1;
        rgledoost[2].leds.R1 = rgledoost[2].leds.R2 = 1;
    }

    /* Check to see if the Green LEDs should be on for the bottom color.
    */
    if (( colorGreen == colorBottom ) ||
        ( colorYellow == colorBottom ) ||
        ( colorCyan == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[0].leds.G3 = rgledoost[0].leds.G4 = 1;
        rgledoost[2].leds.G1 = rgledoost[2].leds.G2 = 1;
    }

    /* Check to see if the Blue LEDs should be on for the bottom color.
    */
    if (( colorBlue == colorBottom ) ||
        ( colorFuchsia == colorBottom ) ||
        ( colorCyan == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[0].leds.B3 = rgledoost[0].leds.B4 = 1;
        rgledoost[2].leds.B1 = rgledoost[2].leds.B2 = 1;
    }

    /* We need to remain in state 0 and state 2 for the amount of time specified
    ** by tmsOn, as these are the states during which the LEDs are turned on.
    ** We will remain in state 1 and 4 for the amount of time specified as
    ** tmsDelay, which may create a time during which all LEDs are turned off.
    */
    rgledoost[0].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[1].ctickSt = LedMsToTicks(tmsDelay);
    rgledoost[2].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[3].ctickSt = LedMsToTicks(tmsDelay);

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedCircularBlink
**
**  Parameters:
**      colorDisp   - color displayed by the LEDs
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**      tmsOn       - number of milliseconds and LED is on before a rotation
**                    oocurs
**      fCC         - fTrue to display LED pattern LD1->LD4->LD3->LD2->LD1
**                  - fFalse to display LED pattern LD1->LD2->LD3->LD4->LD1
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the LEDs
**      will rotate in a circular pattern. One LED is turned on for tmsOn
**      milliseconds. After tmsOn milliseconds have elapsed the current LED
**      turns off and a clockwise or counter-clockwise rotation occurs, and then
**      the next LED turns on. At any given time only a single LED is turned on.
**      The brightness of the LEDs can be increased or decreased by increasing
**      or decreasing the value specified for dtcMaster. Specifying a duty cycle
**      of 0 will result in all LEDs remaining off.
*/
void
LedCircularBlink(BYTE colorDisp, WORD dtcMaster, WORD tmsOn, BOOL fCC) {
    
    BYTE    i;
    WORD    ctick;
    
    ctick = LedMsToTicks(tmsOn);
    
    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = ctick;
        rgledoost[i].fsFlags = 0;
    }

    if ( fCC ) {
        /* LED pattern is counter-clockwise (LD1->LD4->LD3->LD2->LD1).
        */
        rgledoost[0].leds.R1 = rgledoost[0].leds.G1 = rgledoost[0].leds.B1 = 1;
        rgledoost[1].leds.R4 = rgledoost[1].leds.G4 = rgledoost[1].leds.B4 = 1;
        rgledoost[2].leds.R3 = rgledoost[2].leds.G3 = rgledoost[2].leds.B3 = 1;
        rgledoost[3].leds.R2 = rgledoost[3].leds.G2 = rgledoost[3].leds.B2 = 1;
    }
    else {
        /* LED pattern is clockwise (LD1->LD2->LD3->LD4->LD1).
        */
        rgledoost[0].leds.R1 = rgledoost[0].leds.G1 = rgledoost[0].leds.B1 = 1;
        rgledoost[1].leds.R2 = rgledoost[1].leds.G2 = rgledoost[1].leds.B2 = 1;
        rgledoost[2].leds.R3 = rgledoost[2].leds.G3 = rgledoost[2].leds.B3 = 1;
        rgledoost[3].leds.R4 = rgledoost[3].leds.G4 = rgledoost[3].leds.B4 = 1;
    }

    /* Set the duty cycle for the Red, Green, and Blue LEDs appropriately
    ** to display the specified color.
    */
    switch ( colorDisp ) {

        case colorRed:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            break;

        case colorGreen:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorBlue:
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorYellow:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorFuchsia:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorCyan:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorOrange:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = (dtcMaster >> 1);
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = (dtcMaster >> 1);
            break;

        case colorWhite:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorBlack:
        default:
            break;
    }

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedCircularBlinkAdjustRate
**
**  Parameters:
**      tmsOn       - number of milliseconds and LED is on before a rotation
**                    oocurs
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function adjusts the amount of time that the LEDs are on when
**      configured for circular blink. This adjust does NOT reset the LED state
**      machine.
**
**  Notes:
**      This function does not configure the state machine for circular blink.
**      The state machine must be configured for circular blink prior to calling
**      this function.
*/
void
LedCircularBlinkAdjustRate(WORD tmsOn) {

    BYTE    i;
    WORD    ctick;

    ctick = LedMsToTicks(tmsOn);
    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].ctickSt = ctick;
    }
}

/* ------------------------------------------------------------ */
/***	LedCircularBlinkAdjustColor
**
**  Parameters:
**      colorDisp   - color displayed by the LEDs
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function adjusts the color that the LEDs display while configured
**      for circular blink. This function does does NOT reset the LED state
**      machine.
**
**  Notes:
**      This function does not configure the state machine for circular blink.
**      The state machine must be configured for circular blink prior to calling
**      this function.
*/
void
LedCircularBlinkAdjustColor(BYTE colorDisp, WORD dtcMaster) {

    /* Set the duty cycle for the Red, Green, and Blue LEDs appropriately
    ** to display the specified color.
    */
    switch ( colorDisp ) {

        case colorRed:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            break;

        case colorGreen:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorBlue:
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorYellow:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            break;

        case colorFuchsia:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorCyan:
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorOrange:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = (dtcMaster >> 1);
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = (dtcMaster >> 1);
            break;

        case colorWhite:
            rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcMaster;
            rgledoost[2].leds.dtcRed = rgledoost[3].leds.dtcRed = dtcMaster;
            rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcMaster;
            rgledoost[2].leds.dtcGreen = rgledoost[3].leds.dtcGreen = dtcMaster;
            rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcMaster;
            rgledoost[2].leds.dtcBlue = rgledoost[3].leds.dtcBlue = dtcMaster;
            break;

        case colorBlack:
        default:
            break;
    }
}

/* ------------------------------------------------------------ */
/***	LedOppositeBlink
**
**  Parameters:
**      colorTop    - color to set for the top LEDs (LD1 and LD2)
**      colorBottom - color to set for the bottom LEDs (LD3 and LD4)
**      dtcMaster   - master duty cycle (valid values are 0 to 256) for LED PWM
**      tmsOn       - number of milliseconds LEDs are turned on during top or
**                    bottom state
**      tmsDelay    - number of milliseconds LEDs are turned off between top and
**                    bottom state
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the top
**      LEDs (LD1 and LD2) display the specified color for tmsOn milliseconds.
**      After tmsOn milliseconds, the LEDs will all be off for tmsDelay
**      milliseconds. After tmsDelay milliseconds, the bottom LEDs (LD3 and LD4)
**      display the color specified by colorBottom. The LEDs then all turn off
**      and remain off for tmsDelay milliseconds, after which the state machine
**      wraps around to the initial state and the top LEDs turn on again.
**
**  Notes:
**      This function may be used to make the top LEDs or the bottom LEDs blink
**      at a specific color. This can be done by specifying the desired color
**      for either colorTop or colorBottom and then specifying colorBlack for
**      the other color parameter.
*/
void
LedOppositeBlink(BYTE colorTop, BYTE colorBottom, WORD dtcMaster, WORD tmsOn, WORD tmsDelay) {

    BYTE    i;

    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0;
        rgledoost[i].leds.dtcRed = dtcMaster;
        rgledoost[i].leds.dtcGreen = dtcMaster;
        rgledoost[i].leds.dtcBlue = dtcMaster;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].fsFlags = 0;
    }

    /* Check to see if the Red LEDs should be on for the top color.
    */
    if (( colorRed == colorTop ) ||
        ( colorYellow == colorTop ) ||
        ( colorFuchsia == colorTop ) ||
        ( colorOrange == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.R1 = rgledoost[0].leds.R2 = 1;
    }

    /* Check to see if the Green LEDs should be on for the top color.
    */
    if (( colorGreen == colorTop ) ||
        ( colorYellow == colorTop ) ||
        ( colorCyan == colorTop ) ||
            ( colorOrange == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.G1 = rgledoost[0].leds.G2 = 1;
    }

    /* Check to see if the Blue LEDs should be on for the top color.
    */
    if (( colorBlue == colorTop ) ||
        ( colorFuchsia == colorTop ) ||
        ( colorCyan == colorTop ) ||
        ( colorWhite == colorTop )) {
        rgledoost[0].leds.B1 = rgledoost[0].leds.B2 = 1;
    }

    /* Check to see if the Red LEDs should be on for the bottom color.
    */
    if (( colorRed == colorBottom ) ||
        ( colorYellow == colorBottom ) ||
        ( colorFuchsia == colorBottom ) ||
        ( colorOrange == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[2].leds.R3 = rgledoost[2].leds.R4 = 1;
    }

    /* Check to see if the Green LEDs should be on for the bottom color.
    */
    if (( colorGreen == colorBottom ) ||
        ( colorYellow == colorBottom ) ||
        ( colorCyan == colorBottom ) ||
        ( colorOrange == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[2].leds.G3 = rgledoost[2].leds.G4 = 1;
    }

    /* Check to see if the Blue LEDs should be on for the bottom color.
    */
    if (( colorBlue == colorBottom ) ||
        ( colorFuchsia == colorBottom ) ||
        ( colorCyan == colorBottom ) ||
        ( colorWhite == colorBottom )) {
        rgledoost[2].leds.B3 = rgledoost[2].leds.B4 = 1;
    }

    /* If the caller wants to display orange then the duty cycle applied to the
    ** Green LED must be half of that of the Red LED.
    */
    if ( colorOrange == colorTop ) {
        rgledoost[0].leds.dtcGreen = (dtcMaster >> 1);
    }

    if ( colorOrange == colorBottom ) {
        rgledoost[2].leds.dtcGreen = (dtcMaster >> 1);
    }

    /* We need to remain in state 0 and state 2 for the amount of time specified
    ** by tmsOn, as these are the states during which the LEDs are turned on.
    ** We will remain in state 1 and 4 for the amount of time specified as
    ** tmsDelay, which may create a time during which all LEDs are turned off.
    */
    rgledoost[0].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[1].ctickSt = LedMsToTicks(tmsDelay);
    rgledoost[2].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[3].ctickSt = LedMsToTicks(tmsDelay);

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedProgressiveGlow
**
**  Parameters:
**      colorDisp   - color to display on the LEDs
**      tmsStep     - number of milliseconds between duty cycle updates
**      tmsOn       - number of milliseconds the LED duty cycle ramps up
**                  - number of milliseconds the LED duty cycle ramps down when
**                    fTrue is specified for fUpDown
**      tmsOff      - number of mliliseconds the LED remains in the off state
**                    before transitioning back to the on state
**      fUpDown     - fTrue to ramp the LED duty cycle down after it ramps up
**                  - fFalse to turn the LED off without ramping it down
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that all four
**      LEDs will display a glowing pattern. The LEDs will start in an off state
**      and then continue to ramp up the duty cycle until it reaches the maximum
**      allowable duty cycle (256) or tmsOn milliseconds have elapsed. If the
**      LEDs reach their maximum duty cycle then they will remain on at that
**      duty cycle until a total of tmsOn milliseconds have elapsed. After
**      tmsOn milliseconds have elapsed the LEDs will either turn off
**      (fUpDown = fFalse) or the duty cycle will ramp down at a rate of
**      tmsStep. If the LEDs ramp down then they their duty cycle will continue
**      to decrease until it either reaches 0 or tmsOn milliseconds have
**      elapsed. If the duty cycle reaches zero then the LEDs will remain off
**      until tmsOn milliseconds have elapsed. Once this happens the
**      machine will either delay for tmsOff (>0) or it will transition back to
**      the on state and the entire process will repeat indefinitely.
*/
void
LedProgressiveGlow(BYTE colorDisp, WORD tmsStep, WORD tmsOn, WORD tmsOff, BOOL fUpDown) {
    BYTE    i;
    WORD    ctick;

    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0x0FFF;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = 0;
        rgledoost[i].fsFlags = 0;
    }

    ctick = LedMsToTicks(tmsOn);
    rgledoost[0].ctickSt = ctick;
    if ( fUpDown ) {
        rgledoost[1].ctickSt = ctick;
    }

    rgledoost[0].ctickDtc = LedMsToTicks(tmsStep);
    if ( fUpDown ) {
        rgledoost[1].ctickDtc = LedMsToTicks(tmsStep);
    }

    /* Stay in an off state for the amount of time specified by the caller.
    ** Please note that we use state 2 to implement the off delay, as state 1
    ** may be used for a down glow. Since state 3 is the initial state that we
    ** enter it's tick count must always be zero so that we will immediately
    ** advance to state 0 and start the glow on sequence.
    */
    rgledoost[2].ctickSt = LedMsToTicks(tmsOff);

    switch ( colorDisp ) {

        case colorRed:
            rgledoost[0].fGlowR = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowR = 1;
            }
            break;

        case colorGreen:
            rgledoost[0].fGlowG = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowG = 1;
            }
            break;

        case colorBlue:
            rgledoost[0].fGlowB = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowB = 1;
            }
            break;

        case colorYellow:
            rgledoost[0].fGlowR = 1;
            rgledoost[0].fGlowG = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowR = 1;
                rgledoost[1].fGlowG = 1;
            }
            break;

        case colorFuchsia:
            rgledoost[0].fGlowR = 1;
            rgledoost[0].fGlowB = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowR = 1;
                rgledoost[1].fGlowB = 1;
            }
            break;

        case colorCyan:
            rgledoost[0].fGlowG = 1;
            rgledoost[0].fGlowB = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowG = 1;
                rgledoost[1].fGlowB = 1;
            }
            break;

        case colorOrange:
            rgledoost[0].fGlowR = 1;
            rgledoost[0].fGlowG = 1;
            rgledoost[0].fHalfDtcG = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowR = 1;
                rgledoost[1].fGlowG = 1;
                rgledoost[1].fHalfDtcG = 1;
            }
            break;
        case colorWhite:
            rgledoost[0].fGlowR = 1;
            rgledoost[0].fGlowG = 1;
            rgledoost[0].fGlowB = 1;
            if ( fUpDown ) {
                rgledoost[1].fGlowR = 1;
                rgledoost[1].fGlowG = 1;
                rgledoost[1].fGlowB = 1;
            }
            break;

        case colorBlack:
        default:
            break;
    }

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedStrobe
**
**  Parameters:
**      dtcR        - duty cycle for the Red LEDs
**      dtcG        - duty cycle for the Green LEDs
**      dtcB        - duty cycle for the Blue LEDs
**      tmsOn       - number of milliseconds the LED remains in an on state
**      tmsOff      - number of mliliseconds the LEDs remain in the off state
**                    before transitioning back to an on state
**      fRight      - fTrue to display pattern LD1->LD2->OFF and LD4->LD3->OFF
**                  - fFalse to display pattern LD2->LD1->OFF and LD3->LD4->OFF
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the LEDs
**      display a strobing pattern from left to right or right to left. If fTrue
**      is specified for fRight then the LEDs will be turned in the following
**      sequence:
**          1. LD1 and LD4 on for tmsOn milliseconds
**          2. LD2 and LD3 on for tmsOn milliseconds
**          3. All OFF for tmsOff milliseconds
**          4. goto 1
**
**      If fFalse is specified for fRight then the LEDs will be turned on in the
**      following sequence:
**          1. LD2 and LD3 on for tmsOn milliseconds
**          2. LD1 and LD4 on for tmsOn milliseconds
**          3. All OFF for tmsOff milliseconds
**          4. goto 1
**
**      The color of the LEDs is set by specifying the duty cycle for the Red,
**      Green, and Blue LEDs.
**
**  Notes:
**      If 0 or a very small value is specified for tmsOff then it will be
**      difficult, if not impossible, to tell which direction the LEDs are
**      strobing. In this case it will appear as if the LEDs are just alterating
**      between each side.
*/
void
LedStrobe(WORD dtcR, WORD dtcG, WORD dtcB, WORD tmsOn, WORD tmsOff, BOOL fRight) {

    BYTE    i;

    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = 0;
        rgledoost[i].fsFlags = 0;
    }

    /* To strobe left LED's LD2 and LD3 should be on during state 0. LD1 and LD4
    ** should be on during state 1. All LEDs should be off in all other states.
    ** The opposite is true if we want to strobe the LEDs to the right.
    */
    if ( fRight ) {
        rgledoost[0].leds.R1 = rgledoost[0].leds.G1 = rgledoost[0].leds.B1 = 1;
        rgledoost[0].leds.R4 = rgledoost[0].leds.G4 = rgledoost[0].leds.B4 = 1;
        rgledoost[1].leds.R2 = rgledoost[1].leds.G2 = rgledoost[1].leds.B2 = 1;
        rgledoost[1].leds.R3 = rgledoost[1].leds.G3 = rgledoost[1].leds.B3 = 1;
    }
    else {
        rgledoost[0].leds.R2 = rgledoost[0].leds.G2 = rgledoost[0].leds.B2 = 1;
        rgledoost[0].leds.R3 = rgledoost[0].leds.G3 = rgledoost[0].leds.B3 = 1;
        rgledoost[1].leds.R1 = rgledoost[1].leds.G1 = rgledoost[1].leds.B1 = 1;
        rgledoost[1].leds.R4 = rgledoost[1].leds.G4 = rgledoost[1].leds.B4 = 1;
    }

    rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcR;
    rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcG;
    rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcB;

    /* We need to remain in the first two states for the amount of time
    ** specified by tmsOn. We will remain in the third state, which is an off
    ** state, for tmsOff milliseconds. The fourth state is a dummy state that
    ** we will move through immediately.
    */
    rgledoost[0].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[1].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[2].ctickSt = LedMsToTicks(tmsOff);

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedStrobeOpposite
**
**  Parameters:
**      dtcR        - duty cycle for the Red LEDs
**      dtcG        - duty cycle for the Green LEDs
**      dtcB        - duty cycle for the Blue LEDs
**      tmsOn       - number of milliseconds the LED remains in an on state
**      tmsOff      - number of mliliseconds the LEDs remain in the off state
**                    before transitioning back to an on state
**      fInvert     - fTrue to display pattern LD1->LD2->OFF and LD3->LD4->OFF
**                  - fFalse to display pattern LD2->LD1->OFF and LD4->LD3->OFF
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function configures the LED on/off state machine such that the LEDs
**      display a strobing pattern from left to right or right to left, with the
**      top and bottom LEDs strobing in opposite directions. If fTrue is
**      specified for fInvert then the LEDs will be turned in the following
**      sequence:
**          1. LD1 and LD3 on for tmsOn milliseconds
**          2. LD2 and LD4 on for tmsOn milliseconds
**          3. All OFF for tmsOff milliseconds
**          4. goto 1
**
**      If fFalse is specified for fInvert then the LEDs will be turned on in
**      the following sequence:
**          1. LD2 and LD4 on for tmsOn milliseconds
**          2. LD1 and LD3 on for tmsOn milliseconds
**          3. All OFF for tmsOff milliseconds
**          4. goto 1
**
**      The color of the LEDs is set by specifying the duty cycle for the Red,
**      Green, and Blue LEDs.
**
**  Notes:
**      If 0 or a very small value is specified for tmsOff then it will be
**      difficult, if not impossible, to tell which direction the LEDs are
**      strobing. In this case it will appear as if the LEDs are just alterating
**      between corners.
*/
void
LedStrobeOpposite(WORD dtcR, WORD dtcG, WORD dtcB, WORD tmsOn, WORD tmsOff, BOOL fInvert) {

    BYTE    i;

    for ( i = 0; i < cledoostMax; i++ ) {
        rgledoost[i].leds.leds = 0;
        rgledoost[i].leds.dtcRed = 0;
        rgledoost[i].leds.dtcGreen = 0;
        rgledoost[i].leds.dtcBlue = 0;
        rgledoost[i].ctickDtc = 0;
        rgledoost[i].ctickSt = 0;
        rgledoost[i].fsFlags = 0;
    }

    /* To strobe the top LEDs (LD1 and LD2) to the left LD2 is on during state
    ** 0 and LD1 is on during state 1. To strobe the bottom LEDs (LD3 and LD4)
    ** to the right LD4 is on during state 0 and LD3 is on during state 1. The
    ** opposite is true if we are inverting.
    */
    if ( fInvert ) {
        rgledoost[0].leds.R1 = rgledoost[0].leds.G1 = rgledoost[0].leds.B1 = 1;
        rgledoost[0].leds.R3 = rgledoost[0].leds.G3 = rgledoost[0].leds.B3 = 1;
        rgledoost[1].leds.R2 = rgledoost[1].leds.G2 = rgledoost[1].leds.B2 = 1;
        rgledoost[1].leds.R4 = rgledoost[1].leds.G4 = rgledoost[1].leds.B4 = 1;

    }
    else {
        rgledoost[0].leds.R2 = rgledoost[0].leds.G2 = rgledoost[0].leds.B2 = 1;
        rgledoost[0].leds.R4 = rgledoost[0].leds.G4 = rgledoost[0].leds.B4 = 1;
        rgledoost[1].leds.R1 = rgledoost[1].leds.G1 = rgledoost[1].leds.B1 = 1;
        rgledoost[1].leds.R3 = rgledoost[1].leds.G3 = rgledoost[1].leds.B3 = 1;
    }

    rgledoost[0].leds.dtcRed = rgledoost[1].leds.dtcRed = dtcR;
    rgledoost[0].leds.dtcGreen = rgledoost[1].leds.dtcGreen = dtcG;
    rgledoost[0].leds.dtcBlue = rgledoost[1].leds.dtcBlue = dtcB;

    /* We need to remain in the first two states for the amount of time
    ** specified by tmsOn. We will remain in the third state, which is an off
    ** state, if fDelay is set. The fourth state is a dummy state that we will
    ** move through immediately.
    */
    rgledoost[0].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[1].ctickSt = LedMsToTicks(tmsOn);
    rgledoost[2].ctickSt = LedMsToTicks(tmsOff);

    /* Update the state variables so that we start off in the last on/off state
    ** but immediately transition to the first state, which will cause the LEDs
    ** to be set the correct state.
    */
    iledoost = cledoostMax - 1;
    ctickSt = 0;
    ctickDtc = 0;
    idtcGlow = 0;
}

/* ------------------------------------------------------------ */
/***	LedCalibrationStart
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fCalStart flag in the LED flags variable, which
**      will tell the state machine that a calibration proceedure was just
**      started. The fCalSuccess and fCalFailed flags are also cleared.
**
**  Notes:
**      The fCalStart flag will be cleared by the LED state machine once it is
**      processed as part of a call to the LedTick() function. Please note that
**      depending on how this function and LedTick() are called (ISR context vs
**      mainline code), it's possible that a race condition could occur.
*/
void
LedCalibrationStart() {

    ledflgs.fCalStart = 1;
    ledflgs.fCalSuccess = 0;
    ledflgs.fCalFailed = 0;
}

/* ------------------------------------------------------------ */
/***	LedCalibrationSuccess
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fCalSuccess flag in the LED flags variable, which
**      will tell the state machine that the previously initiated calibration
**      proceedure was successfully completed.
**
**  Notes:
**      The fCalSuccess flag will be cleared by the LED state machine once it is
**      processed as part of a call to the LedTick() function. Please note that
**      depending on how this function and LedTick() are called (ISR context vs
**      mainline code), it's possible that a race condition could occur.
*/
void
LedCalibrationSuccess() {

    ledflgs.fCalSuccess = 1;
}

/* ------------------------------------------------------------ */
/***	LedCalibrationSuccess
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fCalFailed flag in the LED flags variable, which
**      will tell the state machine that the previously initiated calibration
**      proceedure failed.
**
**  Notes:
**      The fCalFailed flag will be cleared by the LED state machine once it is
**      processed as part of a call to the LedTick() function. Please note that
**      depending on how this function and LedTick() are called (ISR context vs
**      mainline code), it's possible that a race condition could occur.
*/
void
LedCalibrationFailed() {

    ledflgs.fCalFailed = 1;
}

/* ------------------------------------------------------------ */
/***	LedDispBrakeCoast
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fDispBC flag in the LED flags variable, which
**      will tell the state machine to display the current Brake/Coast mode.
**
**  Notes:
**      The fDispBC flag will be cleared by the LED state machine once it is
**      processed as part of a call to the LedTick() function. Please note that
**      depending on how this function and LedTick() are called (ISR context vs
**      mainline code), it's possible that a race condition could occur.
*/
void
LedDispBrakeCoast() {

    ledflgs.fDispBC = 1;
}

/* ------------------------------------------------------------ */
/***	LedRestoreCalStart
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fRestoreCalStart flag in the LED flags variable,
**      which will tell the state machine that we just detected that the user
**      wants to restore the servo calibration constants. The
**      fRestoreCalComplete flag is also cleared.
**
**  Notes:
**      The fRestoreCalStart flag will be cleared by the LED state machine once
**      it is processed as part of a call to the LedTick() function. Please note
**      that depending on how this function and LedTick() are called (ISR
**      context vs mainline code), it's possible that a race condition could
**      occur.
*/
void
LedRestoreCalStart() {

    ledflgs.fRestoreCalStart = 1;
    ledflgs.fRestoreCalComplete = 0;
}

/* ------------------------------------------------------------ */
/***	LedRestoreCalComplete
**
**  Parameters:
**      none
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fRestoreCalComplete flag in the LED flags
**      variable, which will tell the state machine that the user has released
**      the BC button and that the servo calibration constants have been
**      restored.
**
**  Notes:
**      The fRestoreCalComplete flag will be cleared by the LED state machine
**      once it is processed as part of a call to the LedTick() function. Please
**      note that depending on how this function and LedTick() are called (ISR
**      context vs mainline code), it's possible that a race condition could
**      occur.
*/
void
LedRestoreCalComplete() {

    ledflgs.fRestoreCalComplete = 1;
}

/* ------------------------------------------------------------ */
/***	LedInitComplete
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if the LED initialization state has completed, fFalse otherwise
**
**  Errors:
**      none
**
**  Description:
**      This function returns the status of LED initialization. During
**      initialization the LED state machine is configured to perform a
**      progressive glow of the LEDs for some amount of time, which indicates to
**      the user that the motor controller is powering up. During this time the
**      LED state machine ignores external inputs that may change the state of
**      the state machine and this function returns fFalse. Once initializatoin
**      has been completed this function will return fTrue.
*/
BOOL    
LedInitComplete() {

    return ( ledflgs.fInitComplete ) ? fTrue : fFalse;
}

/******************************************************************************/
