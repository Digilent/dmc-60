/************************************************************************/
/*									*/
/*  Ctrlr.c - Motor Controller Control Loop and State Functions         */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to    */
/*  configure and control the state of the Motor Controller.            */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/13/2016 (MichaelA): created			                */
/*  06/15/2016 (MichaelA): added a definition for a default voltage     */
/*      ramp rate in order to allow the default ramp rate to be non     */
/*      zero on the DMC1 and DMC2                                       */
/*  06/24/2016 (MichaelA): reduced tmpDTCMax and tmpDTCMin by 10        */
/*      degrees after observing a DMC2 burn up almost immediately after */
/*      temperature based scaling kicked in                             */
/*  07/15/2016 (MichaelA): fixed a bug in CtrlrSignalFault that         */
/*      resulted in the fault flag being set when a COMM fault is       */
/*      signaled. This bug prevented fast recovery when the link was    */
/*      re-established quickly after having been lost.                  */
/*  11/02/2016 (MichaelA): changed default ramp rate to 1024 to improve */
/*      response time                                                   */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Cmd.h"
#include "HBridge.h"
#include "Servo.h"
#include "Adc.h"
#include "Led.h"
#include "Ctrlr.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define the default number of ticks that are required to clear
** all faults when a new fault occurs.
*/
#define ctickClearFaultsDefault     6000 // 3 seconds

/* Define the states that the controller can be in.
*/
#define ctrlstWaitForLink           0
#define ctrlstRun                   1
#define ctrlstFault                 2

#if defined(DEAD)
/* Define the ambient temperature (in degrees C) at which we will scale back
** the duty cycle in an attempt to reduce the temperature. Please note that
** temperature thresholds and output reduction rates are defined in as signed
** 16.16 fixed point values.
*/
//#define tmpReduceDTC      0x00500000  // 80.0 degrees C
//#define tmpReduceDTC        0x00230000  // 35.0 degrees C

#define mplrReduceDTC       0x0000C000  // 0.75
#define citrReduceDTCMax    2000       // stay below temp for 1 seconds before
                                        // re-enabling full throttle

#define tmpReduceDTC        0x5000      // 80.0 degrees C
#define mplrDTCIncrease     256         // amount to increase multiplier by
#define mplrDTCDecrease     256         // amount to decrease multiplier by
#define citrMplrMax         250         // update multiplier every 125ms
#endif

/* Define the ambient temperatures (in degrees C) corresponding to minimum and
** maximum duty cycle multipliers. Please note that the output duty cycle will
** be 100% of the calculated target at temperatures below mplrDTCMax. Also note
** that temperature thresholds are defined as signed 8.8 fixed point numbers.
*/
#if defined(DEAD)
#define tmpDTCMax           0x4600      // 70.0C temperature below which DTC multiplier is at its max value
#define tmpDTCMin           0x6400      // 100.0C temperature at which DTC multiplier is at it's minimum value
#else
#define tmpDTCMax           0x5500      // 85.0C temperature below which DTC multiplier is at its max value
#define tmpDTCMin           0x6900      // 105.0C temperature at which DTC multiplier is at it's minimum value
#endif

/* Define the minimum and maximum duty cycle multipliers. These are used in
** conjunction with the above temperature thresholds to calculate the slope of
** the equation used for determining the duty cycle multiplier associated with a
** sepecific temperature. Please note that the duty cycle multipliers are defined
** as signed 16.16 fixed point numbers.
*/
#define mplrDTCMax          0x00010000  // maximum duty cycle muptilier is 1.0
#define mplrDTCMin          0x00000000  // minimum duty cycle multiplier is 0.0

/* Define the default voltage ramp rate. The voltage ramp rate controls how
** quickly the voltage is increased or decreased in Voltage Control Mode. Please
** note that a value of 0 will disable voltage ramping, meaning that the current
** voltage will be set to the target voltage each time the control function is
** called.
*/
#define vltgRampRateDefalt  1024 // takes 32ms to go from full forward to full reverse

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* Define variable to keep track of the link state.
*/
static volatile BYTE    linktCur = linktNone;

/* Define variables to keep track of the current and target
** bridge voltage when operating in voltage control mode.
*/
static volatile int16_t vltgCur = 0;
static volatile int16_t vltgTarget = 0;
static volatile WORD    vltgRampRate = vltgRampRateDefalt;

/* Define the variable used to keep track of the control state.
*/
static volatile WORD    ctrlstCur = ctrlstWaitForLink;

/* Define the variables used to keep track of the control flags.
*/
static volatile CTRLRFLAGS  ctrlflgs;

/* Define the variables used to keep track of the current fault
** status and fault counts.
*/
static volatile DWORD   ctickFaults = 0;
static volatile DWORD   ctickClearFaults = ctickClearFaultsDefault;
static volatile WORD    fsFaults = 0;
static volatile WORD    fsStickyFaults = 0;
static volatile BYTE    cfltOverCurrent = 0;
static volatile BYTE    cfltOverTemp = 0;
static volatile BYTE    cfltUnderVoltage = 0;
static volatile BYTE    cfltGateDriver = 0;
static volatile BYTE    cfltComm = 0;

#if defined(DEAD)
/* Declare the variables used to keep track of a reduction to the output
** duty cycle. This may occur as a result of a pre-defined temperature
** threshold being exceeded.
*/

static volatile WORD    citrReduceDTC = 0;

static volatile int32_t mplrDtc = mplrDTCMax;
static volatile WORD    citrMplr = 0;
static volatile int16_t tmpPrev = 0;
#endif

/* Declare the variables used for calculating the duty cycle multiplier that
** corresponds to a temperature value.
*/
static volatile int32_t mplrDtcSlope = 0;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/



void
CtrlrControlLoop() {

    switch ( ctrlstCur ) {
        case ctrlstWaitForLink:
            /* Don't do anything until after ADC calibration and LED
            ** initialization have completed.
            */
            if (( ! AdcCalibrationComplete() ) || ( ! LedInitComplete() )) {
                break;
            }

            /* Check to see if a new fault has occured.
            */
            if ( ctrlflgs.fFault ) {
                /* A new fault has occured. Reset the fault tick counter and
                ** clear out the fault flag.
                */
                ctrlflgs.fFault = 0;
                ctickFaults = ctickClearFaults;
            }

            /* Decrement the fault count as necessary. If it reaches zero, then
            ** clear the fault flags.
            */
            if ( 0 < ctickFaults ) {
                ctickFaults--;
                if ( 0 == ctickFaults ) {
                    fsFaults = 0;
                }
            }

            /* Check to see if we have a valid link and if so, then advance to
            ** the appropriate state.
            */
            if ( linktNone != linktCur ) {
                if ( 0 < ctickFaults ) {
                    ctrlstCur = ctrlstFault;
                }
                else {
                    ctrlstCur = ctrlstRun;
                }
            }

            break;

        case ctrlstRun:
            /* Process any commands that are in the command queue. This may
            ** result in the target voltage being changed.
            */
            CmdProcessQueue(fFalse);

            /* If the controller isn't halted then run the appropriate PID code
            ** based on the current control mode. This will result in the output
            ** voltage being adjusted as necessary.
            */
            if ( 0 == ctrlflgs.fHalt ) {
                /* M00TODO: add variable containing the mode and use it to
                ** decide which control function to call.
                */
                CtrlrVoltageMode();
            }

            /* Check to see if we have lost the control link.
            */
            if ( linktNone == linktCur ) {
                /* Lost the control link. Force the output voltage to 0, abort
                ** any calibration proceedure that may be in progress, signal a
                ** COMM fault and then wait for a new link to be established.
                */
                CtrlrForceNeutral();
                ServoAbortCalibration();
                CtrlrSignalFault(fltComm);
                ctrlstCur = ctrlstWaitForLink;
            }

            /* Check to see if a fault has occured.
            */
            if ( ctrlflgs.fFault ) {
                /* A  fault has occured. Reset the fault tick counter and
                ** clear out the fault flag.
                */
                ctrlflgs.fFault = 0;
                ctickFaults = ctickClearFaults;

                /* Force the output voltage to 0 and abort any calibration
                ** proceedure that may be in progress.
                */
                CtrlrForceNeutral();
                ServoAbortCalibration();

                /* proceed to the fault state.
                */
                ctrlstCur = ctrlstFault;
            }

            break;

        case ctrlstFault:
            /* Check to see if a new fault has occured.
            */
            if ( ctrlflgs.fFault ) {
                /* A new fault has occured. Reset the fault tick counter and
                ** clear out the fault flag.
                */
                ctrlflgs.fFault = 0;
                ctickFaults = ctickClearFaults;
            }

            /* Process any commands in the command queue, ignoring the ones that
            ** would result in a change to the H-bridge voltage.
            */
            CmdProcessQueue(fTrue);

            /* Decrement the fault count. If it reaches zero, then clear the
            ** the fault flags and advance to the appropriate state.
            */
            ctickFaults--;
            if ( 0 == ctickFaults ) {
                fsFaults = 0;

                if ( linktNone != linktCur ) {
                    ctrlstCur = ctrlstRun;
                }
                else {
                    ctrlstCur = ctrlstWaitForLink;
                }
            }

            break;
            
        default:
            /* M00TODO: figure out which variables would need to be reset before
            ** we can safely transition from an unknown state to the wait for
            ** link state.
            */
            ctrlstCur = ctrlstWaitForLink;
            break;
    }

    /* Perform an H-Bridge tick. This will apply adjust the duty cycle of the
    ** PWM outputs that control the bridge driver, applying the target voltage
    ** to the load.
    */
    HBridgeTick();
}

/* ------------------------------------------------------------ */
/***	CtrlrInit
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
**      This function initializes the variables used to control the state
**      of the motor controller.
*/
void
CtrlrInit() {

    /* Set the initial link type to linktNone, as no link should be established
    ** at this point.
    */
    linktCur = linktNone;

    /* Set the current and target voltage to 0 and disable voltage ramping.
    */
    vltgCur = 0;
    vltgTarget = 0;
    vltgRampRate = vltgRampRateDefalt;

    /* Set the control state to the wait for link state, as no link should be
    ** established at this point.
    */
    ctrlstCur = ctrlstWaitForLink;

    /* Clear the controller flags.
    */
    ctrlflgs.fsFlags = 0;

    /* Set the variables that keep track of faults to their default values.
    */
    ctickFaults = 0;
    ctickClearFaults = ctickClearFaultsDefault;
    fsFaults = 0;
    fsStickyFaults = 0;
    cfltOverCurrent = 0;
    cfltOverTemp = 0;
    cfltUnderVoltage = 0;
    cfltGateDriver = 0;
    cfltComm = 0;

#if defined(DEAD)
    /* Set the variables used to keep track of a duty cycle reduction to their
    ** default values.
    */
    citrReduceDTC = 0;

    mplrDtc = mplrDTCMax;
    citrMplr = 0;
    tmpPrev = 0;
#endif

    /* Calculate the slope of the duty cycle multiplier. Please note that we
    ** arrange the equation such that the slope should always be positive.
    ** Trying to perform the calculation to produce a negative slope will not
    ** work using integer math if the numerator ends up being smaller than the
    ** denominator. The result of this calculation should be a 16.16 fixed point
    ** number.
    */
    mplrDtcSlope = ((((int32_t)mplrDTCMax) - ((int32_t)mplrDTCMin)) << 8) / (((int32_t)tmpDTCMin) - ((int32_t)tmpDTCMax));
}

/* ------------------------------------------------------------ */
/***	CtrlrSetLinkType
**
**  Parameters:
**      linktSet    - link type to set
**                  - valid link types are linktNone, linktServo, and linktCAN
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the current motor link type. The motor controller may
**      have a single link established or no link at all. Specifying an invalid
**      link type results in the current link type being set to linktNone, which
**      will cause the controller to halt the motors.
*/
void
CtrlrSetLinkType(BYTE linktSet) {

    if (( linktNone == linktSet ) ||
        (( linktServo != linktSet ) && ( linktCAN != linktSet ))) {
        linktCur = linktNone;
    }
    else {
        linktCur = linktSet;
    }
}

/* ------------------------------------------------------------ */
/***	CtrlrGetLinkType
**
**  Parameters:
**      none
**
**  Return Value:
**      current link type for the motor controller, linktNone if no link is
**      established
**
**  Errors:
**      none
**
**  Description:
**      This function returns the current link type. If no link is established
**      then linktNone will be returned.
*/
BYTE
CtrlrGetLinkType() {

    return linktCur;
}

/* ------------------------------------------------------------ */
/***	CtrlrForceNeutral
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
**      Set the current voltage and target voltage to 0 and then force the
**      HBRDIGE voltage to the neutral voltage.
*/
void
CtrlrForceNeutral() {

    /* Set the current voltage and the target voltage to 0.
    */
    vltgCur = 0;
    vltgTarget = 0;

    /* Set the HBRIDGE votlage to the neutral voltage.
    */
    HBridgeSetVoltage(0);

    /* M00TODO: reset PID controllers?
    */
}

/* ------------------------------------------------------------ */
/***	CtrlrSetVoltage
**
**  Parameters:
**      vltgSet - set the H-Bridge target voltage
**              - a value of 32767 corresponds to the maximum positive voltage
**              - a value of -32768 correspodns to the maximum negative voltage
**              - a value of 0 corresponds to a neutral voltage or 0 volts
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      Set the motor controller target voltage. Please note that this does NOT
**      perform an immediate update of the actual voltage being output by the
**      H-Bridge.
*/
void
CtrlrSetVoltage(int16_t vltgSet) {

    vltgTarget = vltgSet;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetVoltage
**
**  Parameters:
**      none
**
**  Return Value:
**      16-bit signed number corresponding to the current H-Bridge voltage
**
**  Errors:
**      none
**
**  Description:
**      Get the current motor controller H-Bridge voltage.
*/
int16_t
CtrlrGetVoltage() {

    return vltgCur;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetVoltage
**
**  Parameters:
**      none
**
**  Return Value:
**      16-bit signed number corresponding to the H-Bridge target voltage
**
**  Errors:
**      none
**
**  Description:
**      Get the current motor controller H-Bridge target voltage. Depending on
**      the control mode and configuration parameters, the target voltage may be
**      different than the current voltage being applied to the H-Bridge.
*/
int16_t
CtrlrGetTargetVoltage() {

    return vltgTarget;
}

/* ------------------------------------------------------------ */
/***	CtrlrSetVoltageRampRate
**
**  Parameters:
**      vltgRampSet - 16-bit value corresponding to the voltage ramp rate
**                  - a value of 0 disables voltage ramping
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      Set the voltage ramp rate used for adjusting the H-Bridge voltage when
**      operating in voltage control mode.
*/
void
CtrlrSetVoltageRampRate(WORD vltgRampSet) {

    vltgRampRate = vltgRampSet;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetVoltageRampRate
**
**  Parameters:
**      none
**
**  Return Value:
**      16-bit unsigned number corresponding to the voltage ramp rate used
**      during voltage control mode
**
**  Errors:
**      none
**
**  Description:
**      Get the voltage ramp rate that's applied when updating the H-Bridge
**      voltage while operating in voltage control mode.
*/
WORD
CtrlrGetVoltageRampRate() {

    return vltgRampRate;
}

/* ------------------------------------------------------------ */
/***	CtrlrSetFaultTime
**
**  Parameters:
**      tmsFault    - number of milliseconds to wait before clearing any
**                    existing faults
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      Set the time, in milliseconds, that the control waits before clearing
**      any faults that occur. This is the minimum amount of time that the
**      controller will remain in the fault state after a new fault occurs.
*/
void
CtrlrSetFaultTime(WORD tmsFault) {

    ctickClearFaults = CtrlrMsToTicks(tmsFault);
}

/* ------------------------------------------------------------ */
/***	CtrlrGetFaultTime
**
**  Parameters:
**      none
**
**  Return Value:
**      number of milliseconds that it takes to clear a fault
**
**  Errors:
**      none
**
**  Description:
**      This function returns a 16-bit value containing the number of
**      milliseconds that the controller is currently programmed to remain in
**      the fault state after a new fault occurs.
*/
WORD
CtrlrGetFaultTime() {

    return CtrlrTicksToMs(ctickClearFaults);
}

/* ------------------------------------------------------------ */
/***	CtrlrSignalFault
**
**  Parameters:
**      fltSignal   - fault being signaled
**                  - do NOT logically or faults as it will mess up the fault
**                    counts
**                  - a comm fault will not cause the fault flag to be set
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function sets the fault specified by fltSignal in the fsFaults and
**      fsStickyFaults variables and then sets the fault flag, fFault in the
**      control flags variable. If this is a new occurance of the fault, meaning
**      that the specified fault isn't presently active, then the fault count
**      associated with that fault will be incremented.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CtrlrSignalFault(WORD fltSignal) {

    BOOL    fSet;

    /* Check to see if this fault is already active or if it's a new fault.
    */
    fSet = ( fsFaults & fltSignal & (~(fltComm)) ) ? fTrue : fFalse;

    /* Set the flag in the fault flags field set.
    */
    fsFaults |= fltSignal & (~(fltComm));
    fsStickyFaults |= fltSignal;

    /* If this is the first time the fault has been set then we need to update
    ** its associated fault count.
    */
    if ( ! fSet ) {
        switch ( fltSignal ) {
            case fltOverCurrent:
                if ( 255 > cfltOverCurrent ) {
                    cfltOverCurrent++;
                }
                break;

            case fltOverTemp:
                if ( 255 > cfltOverTemp ) {
                    cfltOverTemp++;
                }
                break;

            case fltUnderVoltage:
                if ( 255 > cfltUnderVoltage ) {
                    cfltUnderVoltage++;
                }
                break;

            case fltGateDriver:
                if ( 255 > cfltGateDriver ) {
                    cfltGateDriver++;
                }
                break;

            case fltComm:
                if ( 255 > cfltComm ) {
                    cfltComm++;
                }
                break;

            default:
                break;
        }
    }

    /* Set the fault flag to tell the control loop that a new fault has occured.
    ** Please note that if a COMM fault is the only active fault then we don't
    ** want to set the fault flag because doing so would cause the control loop
    ** to enter the fault state and stay there for the full fault time. A loss
    ** off COMM signal shouldn't cause the loop to ignore a new signal.
    */
    if ( fltComm != fltSignal ) {
        ctrlflgs.fFault = 1;
    }
}

/* ------------------------------------------------------------ */
/***	CtrlrGetFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      returns a copy of the field set containing the current fault state of
**      the controller
**
**  Errors:
**      none
**
**  Description:
**      This function returns a copy of the field set representing the current
**      fault state of the motor controller.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
WORD
CtrlrGetFaults() {

    return fsFaults;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetFaults
**
**  Parameters:
**      fClear  - fTrue to clear the sticky faults after reading them
**              - fFalse to maintain the sticky faults after reading them
**
**  Return Value:
**      returns a copy of the field set containing the controller sticky faults
**
**  Errors:
**      none
**
**  Description:
**      This function returns a copy of the field set containing the current
**      controller sticky faults.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
WORD
CtrlrGetStickyFaults(BOOL fClear) {

    WORD    fs;

    fs = fsStickyFaults;

    if ( fClear ) {
        fsStickyFaults = 0;
    }

    return fs;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetCurrentFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      variable containing the number of times that an over current fault has
**      occured since the last time the count was reset
**
**  Errors:
**      none
**
**  Description:
**      This function returns a variable containing the count of times that an
**      an over current fault has occured since the last time the count was
**      reset.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BYTE
CtrlrGetCurrentFaults() {

    return cfltOverCurrent;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetOverTempFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      variable containing the number of times that an over temperature fault
**      has occured since the last time the count was reset
**
**  Errors:
**      none
**
**  Description:
**      This function returns a variable containing the count of times that an
**      an over temperature fault has occured since the last time the count was
**      reset.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BYTE
CtrlrGetOverTempFaults() {

    return cfltOverTemp;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetUVFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      variable containing the number of times that a bus undervoltage fault
**      has occured since the last time the count was reset
**
**  Errors:
**      none
**
**  Description:
**      This function returns a variable containing the count of times that an
**      a bus undervoltage fault has occured since the last time the count was
**      reset.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BYTE
CtrlrGetUVFaults() {

    return cfltUnderVoltage;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetGDriverFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      variable containing the number of times that a gate driver fault
**      has occured since the last time the count was reset
**
**  Errors:
**      none
**
**  Description:
**      This function returns a variable containing the count of times that an
**      a gate driver fault has occured since the last time the count was
**      reset.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BYTE
CtrlrGetGDriverFaults() {

    return cfltGateDriver;
}

/* ------------------------------------------------------------ */
/***	CtrlrGetCommFaults
**
**  Parameters:
**      none
**
**  Return Value:
**      variable containing the number of times that a communications fault
**      has occured since the last time the count was reset
**
**  Errors:
**      none
**
**  Description:
**      This function returns a variable containing the count of times that an
**      a communications fault has occured since the last time the count was
**      reset.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BYTE
CtrlrGetCommFaults() {

    return cfltComm;
}

/* ------------------------------------------------------------ */
/***	CtrlrClearFaultCounts
**
**  Parameters:
**      fsClear - field set specifying which fault counters to clear
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This function resets the count associated with the fault counteres whose
**      associated with is set in fsClear.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CtrlrClearFaultCounts(BYTE fsClear) {

    if ( fltOverCurrent & fsClear ) {
        cfltOverCurrent = 0;
    }

    if ( fltOverTemp & fsClear ) {
        cfltOverTemp = 0;
    }

    if ( fltUnderVoltage & fsClear ) {
        cfltUnderVoltage = 0;
    }

    if ( fltGateDriver & fsClear ) {
        cfltGateDriver = 0;
    }

    if ( fltComm & fsClear ) {
        cfltComm = 0;
    }
}

/* ------------------------------------------------------------ */
/***	CtrlrSetHalt
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
**      This function sets the controller halt flag. While the hatl flag is set
**      the output voltage will NOT be updated.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CtrlrSetHalt() {

    ctrlflgs.fHalt = 1;
}

/* ------------------------------------------------------------ */
/***	CtrlrClearHalt
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
**      This function clears the controller halt flag.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CtrlrClearHalt() {

    ctrlflgs.fHalt = 0;
}

/* ------------------------------------------------------------ */
/***	CtrlrIsHalted
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if the controller is halted, fFalse otherwise
**
**  Errors:
**      none
**
**  Description:
**      This function returns a boolean indicating whether not the controller is
**      presently halted.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BOOL
CtrlrIsHalted() {

    return ( ctrlflgs.fHalt ) ? fTrue : fFalse;
}

/* ------------------------------------------------------------ */
/***	CtrlrIsDTCReduced
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if the duty cycle is being reduced, fFalse otherwise
**
**  Errors:
**      none
**
**  Description:
**      This function returns a boolean indicating whether not the duty cycle
**      is being reduced as a result of the current ambient temperature inside
**      of the motor controller case.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
BOOL
CtrlrIsDTCReduced() {

    return ( ctrlflgs.fReduceDTC ) ? fTrue : fFalse;
}

/* ------------------------------------------------------------ */
/***	CtrlrVoltageMode
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
**      This function implements voltage control mode. It adjusts the output
**      voltage of the H-Bridge, increasing it or decreasing it as necessary to
**      output the target voltage. The ambient temperature inside of the case
**      is monitored, and if it exceeds a pre-defined threshold, then the output
**      duty cycle multiplier is reduced, resulting in a reduction of output
**      voltage. When the ambient temperature inside of the case is below the
**      pre-defined threshold the output duty cycle multiplier is 1.0 and the
**      output voltage is set to the target voltage, or ramped towards the
**      target voltage at the current ramp rate.
**
**  Notes:
**      This function should be called at a fixed rate from the control loop
**      when the device is configured to operate in voltage mode.
**
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
CtrlrVoltageMode() {

    int32_t vltgTemp;
    int16_t vltgTgt;
    int16_t tmpCur;


#if defined(DEAD)
    /* Check to see if the ambient temperature inside of the motor controller
    ** case has exceeded the temperature above which the duty cycle should be
    ** reduced in order to cool down the components.
    */
    if ( fReduceDTC ) {
        /* We preivously reduced the duty cycle due to the temperature threshold
        ** being exceeded. check to see if the temperature has fallen below the
        ** the threshold, and if so, take the appropriate action.
        */
        if ( tmpReduceDTC > AdcGetTemperature() ) {
            citrReduceDTC++;
            if ( citrReduceDTCMax <= citrReduceDTC ) {
                fReduceDTC = fFalse;
                citrReduceDTC = 0;
            }
        }
        else {
            citrReduceDTC = 0;
        }

        vltgTgt = ((((int32_t)vltgTarget) * mplrReduceDTC) >> 16);
    }
    else {
        /* The duty cycle is not currently being limited due to an over
        ** temperature condition. Check to see if the duty cycle should be
        ** limited and adjust the output voltage accordingly.
        */
        if ( tmpReduceDTC <= AdcGetTemperature() ) {
            vltgTgt = ((((int32_t)vltgTarget) * mplrReduceDTC) >> 16);
            fReduceDTC = fTrue;
        }
        else {
            vltgTgt = vltgTarget;
        }
    }
#endif

#if defined(DEAD)
    /* See if we need to check the current ambient temperature inside of the
    ** case and adjust the duty cycle multiplier.
    */
    citrMplr++;
    if ( citrMplrMax <= citrMplr ) {
        citrMplr = 0;
        tmpCur = AdcGetTemperature16();
        if ( tmpReduceDTC <= tmpCur ) {
            /* Has the temperature gone up or staid the same since the last time
            ** we measured it? If so, then we need to further reduce the duty
            ** cycle multiplier, while ensuring that it doesn't fall below the
            ** minimum allowable multiplier value.
            */
            if ( tmpPrev <= tmpCur ) {
                mplrDtc -= mplrDTCDecrease;
                if ( mplrDTCMin > mplrDtc ) {
                    mplrDtc = mplrDTCMin;
                }
            }
        }
        else {
            /* The current temperature is below the temperature at which the
            ** duty cycle multiplier would need to be decreased. Increase the
            ** multiplier if it hasn't already reached the maximum allowable
            ** value.
            */
            if ( mplrDTCMax  > mplrDtc ) {
                mplrDtc += mplrDTCIncrease;
                if ( mplrDTCMax < mplrDtc ) {
                    mplrDtc = mplrDTCMax;
                }
            }
        }

        tmpPrev = tmpCur;
    }

    /* Calculate the target voltage using the current duty cycle multiplier.
    */
    vltgTgt = ((((int32_t)vltgTarget) * mplrDtc) >> 16);
#endif

    tmpCur = AdcGetTemperature16();
    if ( tmpDTCMax < tmpCur ) {
        vltgTgt = ((((int32_t)vltgTarget) * CtrlrMplrFromTemp(tmpCur)) >> 16);
        ctrlflgs.fReduceDTC = 1;
    }
    else {
        vltgTgt = vltgTarget;
        ctrlflgs.fReduceDTC = 0;
    }

    /* If the current voltage matches the target voltage then there's nothing
    ** to do.
    */
    if ( vltgCur != vltgTgt ) {

        if ( 0 == vltgRampRate ) {
            vltgCur = vltgTgt;
        }
        else if ( vltgCur < vltgTgt ) {

            vltgTemp = ((int32_t)vltgCur) + ((int32_t)vltgRampRate);
            if ( ((int32_t)vltgTgt) < vltgTemp ) {
                /* Note: the target can't be more than 32767 so we sature no
                ** higher than 32767 no matter what.
                */
                vltgTemp = ((int32_t)vltgTgt);
            }
            vltgCur = ((int16_t)vltgTemp);
        }
        else {
            vltgTemp = ((int32_t)vltgCur) - ((int32_t)vltgRampRate);
            if ( ((int32_t)vltgTgt) > vltgTemp ) {
                /* Note: the target can't be less than -32768 so we saturate no
                ** lower -32768 no matter what.
                */
                vltgTemp = ((int32_t)vltgTgt);
            }
            vltgCur = ((int16_t)vltgTemp);
        }
    }

    HBridgeSetVoltage(vltgCur);
}

/* ------------------------------------------------------------ */
/***	CtrlrMplrFromTemp
**
**  Parameters:
**      tmpCur  - current ambient temperature in degress C
**              - signed 8.8 fixed point value
**
**  Return Value:
**      signed 16.16 fixed point duty cycle multiplier
**
**  Errors:
**      none
**
**  Description:
**      This function returns the duty cycle multiplier that corresponds to the
**      specified temperature. The multiplier that's returned will be between
**      the values mplrDTCMin and mplrDTCMax.
**
**
**  Notes:
**      This function assumes that the minimum temperature passed in is >=
**      tmpDTCMax.
**
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int32_t
CtrlrMplrFromTemp(int16_t tmpCur) {

    int32_t mplr;

    mplr = ((int32_t)mplrDTCMax) - ((mplrDtcSlope * (tmpCur - tmpDTCMax)) >> 8);
    if ( mplrDTCMin > mplr ) {
        mplr = mplrDTCMin;
    }

    return mplr;
}

/******************************************************************************/
