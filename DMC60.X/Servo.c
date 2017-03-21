/************************************************************************/
/*									*/
/*  Servo.c - Motor Controller Servo (PWM) Input Interface Functions    */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definition of routines that are uesd to        */
/*  implement the servo (pwm) input interface. This includes routines   */
/*  for setting the calibration state, measuring the pulse width, and   */
/*  for mapping an input pulse width to an output voltage.              */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/27/2016 (MichaelA): created			                */
/*  05/24/2016 (MichaelA): modified to read calibration constants from  */
/*      flash during initialization and to write them to flash when     */
/*      ServoEndCalibration is called                                   */
/*  05/25/2016 (MichaelA): fixed bug in ServoInterruptHandler that      */
/*      resulted in an invalid pulse width being calculated after long  */
/*      CPU stalls that occur as the result of flash memory writes      */
/*  05/26/2016 (MichaelA): modified ServoEndCalibration such that the   */
/*      neutral deadband is now 4% of the forward and reverse range     */
/*  06/14/2016 (MichaelA): modified ServoInterruptHandler to add        */
/*      support for the DMC1 and DMC2 through the use of conditional    */
/*      compilation                                                     */
/*  08/19/2016 (MichaelA): modified ServoInterruptHandler such that     */
/*      the leading edge of the input pulse is determined based on      */
/*      whether or not INVERT_SERVO_INPUT is defined                    */
/*  09/02/2016 (MichaelA): added ServoDisable() to allow servo pulse    */
/*      width measurement to be disabled when CAN bus is detected       */
/*  10/12/2016 (MichaelA): modified to call new Cfg functions           */
/*  11/02/2016 (MichaelA): added ServoRestoreCalDefaults() to allow     */
/*      the calibration constants to be restored to their default state */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Cmd.h"
#include "Ctrlr.h"
#include "Led.h"
#include "Servo.h"
#include "Cfg.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* Declare variables used for measuring the pulse width of the
** servo input signal.
*/
static volatile DWORD   cntServoFirstEdge   = 0;

/* Declare variables used for mapping the servo pulse width to
** a voltage.
*/
static volatile SERVOCAL    servocalCur;

/* Declare variables used during servo input pulse width calibration.
*/
static volatile WORD    calstServo          = calstIdle;
static volatile WORD    cntServoMinNew      = 0;
static volatile WORD    cntServoMaxNew      = 0;
static volatile WORD    cntServoNeutralNew  = 0;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	ServoInterruptHandler
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
**      This function is called each time an input capture interrupt occurs on
**      the input capture module (IC1 and IC2) that's used for measuring the
**      servo pulse width. An interrupt should occur on every edge transition
**      that's detected on the servo input pin, producing a new time stamp. Two
**      consecutive edges are then used to calculate the pulse width of the
**      input signal.
**
**  Notes:
**      This function does NOT clear the interrupt flag. It is up to the caller
**      to clear the interrupt flag associated with the interrupt service
**      routine that called this function.
*/
void
ServoInterruptHandler() {
    
    WORD        cbuf;
    DWORD       cnt;
    DWORD       cntPw;
    DWORD       rgcnt[2];
    BOOL        fOverflow;
    BOOL        fFirstEdge;
    static BOOL fWaitForFirstEdge = fFalse;

    /* Check to see if the Input Capture buffer overflowed and set the flag
    ** appropriately. Note: you must perform this check prior to reading from
    ** the buffer, as reading the buffer causes the overflow bit to be cleared.
    */
    fOverflow = ( 1 == IC2CON1bits.ICOV ) ? fTrue : fFalse;

    /* Read the Input Capture buffer and save the two most recent entries.
    */
    cbuf = 0;
    cnt = 0;
    while ( 1 == IC2CON1bits.ICBNE ) {
        cnt = IC1BUF;
        cntPw = IC2BUF;
        cnt |= (cntPw << 16);
        rgcnt[cbuf & 0x0001] = cnt;
        cbuf++;
    }

    /* If we overflowed then we need to discard all timestamps, skip any
    ** additional processing, and set a flag to indicate that the next time this
    ** function is called we don't perform any calculations until after
    ** receiving a new first edge (rising for non-inverted input, falling edge
    ** for inverted input, such as the one produced by a CAN transceiver).
    */
    if ( fOverflow ) {
        fWaitForFirstEdge = fTrue;
        goto lExit;
    }

    /* Determine whether or not the most recent timestamp corresponds to the
    ** first or second edge of the pulse width. For a non-inverted servo input
    ** the first edge is a rising edge. For an inverted servo input the first
    ** edge is a falling edge.
    */
#if defined(INVERT_SERVO_INPUT)
    fFirstEdge = (prtServoIn & (1 << bnServoIn)) ? fFalse : fTrue;
#else
    fFirstEdge = (prtServoIn & (1 << bnServoIn)) ? fTrue : fFalse;
#endif

    /* Check to see if the input capture buffer contained more than one edge.
    */
    if ( 1 < cbuf ) {
        /* The input capture buffer contained more than one edge. If the most
        ** recent edge is the second edge then rgcnt contains the timestamps for
        ** both the first and second edge of the most recent pulse and the width
        ** can be calculated. If the most recent edge is the first edge then
        ** rgcnt contains the timestamps of a second edge from the previous
        ** pulse and the first edge of the current pulse. In this case we will
        ** just end up storing the timestamp for the first edge and then the
        ** next time this function is called we will calculate the pulse width.
        */
        if ( ! fFirstEdge ) {
            cntServoFirstEdge = rgcnt[cbuf & 0x0001];
        }
        fWaitForFirstEdge = fFalse;
    }

    /* Check to see if we are presently in a state where the first edge must be
    ** seen before any further processing can occur.
    */
    if ( fWaitForFirstEdge ) {
        if ( fFirstEdge ) {
            fWaitForFirstEdge = fFalse;
        }
        else {
            goto lExit;
        }
    }

    /* Calculate the pulse width or store the first edge.
    */
    if ( ! fFirstEdge ) {
        /* Detected the second edge. Compute the servo input signal pulse width,
        ** being sure to adjust for any timer overflow that may have occured.
        */
        if ( cntServoFirstEdge <= cnt ) {
            cntPw = cnt - cntServoFirstEdge;
        }
        else {
            cntPw = (0xFFFFFFFF - cntServoFirstEdge) + cnt;
        }

        /* Verify that the pulse width is valid.
        */
        if (( cntServoMinAbsolute <= cntPw ) && ( cntServoMaxAbsolute >= cntPw )) {
            /* The pulse width is valid. Tell the controller that we have a
            ** valid servo link.
            */
            CtrlrSetLinkType(linktServo);

            /* If we aren't presently calibrating or verifying the calibration
            ** constants from a previous calibration operation then we should
            ** send a command to set the output voltage to the voltage that
            ** corresponds to the current pulse width.
            */
            if ( calstIdle == calstServo ) {
                /* There isn't an ongoing calibration proceedure so set the
                ** output voltage to the voltage that corresponds to the
                ** measured pulse width.
                */
                CmdVoltageSet(ServoPwToVoltage(cntPw));
            }
            else if ( calstActive == calstServo ) {
                /* Calibration is presently active. Store the pulse width in an
                ** appropriate variable.
                */
                if ( cntPw < cntServoMinNew ) {
                    cntServoMinNew = cntPw;
                }
                else if ( cntPw > cntServoMaxNew ) {
                    cntServoMaxNew = cntPw;
                }
                else {
                    /* We will likely end up storing many values to
                    ** cntServoNeutralNew during the calibration process. The
                    ** assumption is that the user will input a pulse width
                    ** that corresponds to the neutral position and leave
                    ** the pulse width there long enough that this interrupt
                    ** will be processed one last time before we exit
                    ** calibration mode.
                    */
                    cntServoNeutralNew = cntPw;
                }
            }
        }
        else {
            /* The servo pluse width is not a valid length. Tell the controller
            ** that the link has been lost.
            ** M00TODO: this is going to cause a problem later because we will
            ** set the link type to linktNone even though we may have a valid
            ** CAN link.
            */
            CtrlrSetLinkType(linktNone);
        }
    }
    else {
        /* We just received the first edge. Save the count value associated with
        ** this edge so it can be used later to calculate the pulse width.
        */
        cntServoFirstEdge = cnt;
    }

lExit:

    /* Clear the time out counter and it's associated interrupt flag.
    */
    TMR1 = 0;
    IFS0bits.T1IF = 0;
}

/* ------------------------------------------------------------ */
/***	ServoInit
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
**      This function initializes the Servo Input Interface. It disables the
**      input capture module (IC1 and IC2) and its time base (TMR2), sets all
**      servo related variables to their default state, configures the Servo
**      pin as an input, maps the servo input pin to IC1 and IC2, and it
**      configures the time base and input capture modules required to measure
**      the input.
**
**  Notes:
**      This function does NOT enable the input capture modules or their time
**      base. In order to enable these modules the user must call the
**      ServoInputEnable() function, which may only be called after calling this
**      function.
*/
void
ServoInit() {

    /* Disable the input capture module and its time base.
    */
    IC2CON1bits.ICM = 0; // disable IC2 module
    IC1CON1bits.ICM = 0; // disable IC1 module
    T2CONbits.TON = 0; // disable Timer 2

    /* Set the variables used to measure the pulse width of the servo input
    ** signal to their default values.
    */
    cntServoFirstEdge = 0;

    /* Set the servo variables used for mapping the servo pulse width to
    ** their default value. These will be read from the Flash Based EEPROM. If
    ** the flashed based EEPROM doesn't contain a configuration file then the
    ** variables are initialized to an appropriate default value.
    */
    CfgGetServoCal(&servocalCur);

    /* Set the variables used during servo input pulse width calibration to
    ** their default value.
    */
    calstServo = calstIdle;
    cntServoMinNew = 0xFFFF;
    cntServoMaxNew =  0;
    cntServoNeutralNew = cntServoInvalid;
    
    /* Configure the Servo Pin as an input.
    */
    trsServoIn |= (1 << bnServoIn);

    /* Configure Timer 2 to generate a 1us timebase. This timer will be used as
    ** the clock source for the input capture modules, which are used to measure
    ** the pulse width of the PWM input. Please note that this timer cannot be
    ** started until after the input capture modules have been configured and
    ** enabled.
    */
    T2CONbits.T32 = 0; // disable 32-bit mode
    T2CONbits.TCS = 0; // use internal peripheral bus clock
    T2CONbits.TGATE = 0; // disable gated timer mode
    T2CONbits.TCKPS = 0b10; // 1:64 prescaler
    TMR2 = 0; // clear timer count register
    PR2 = 0xFFFF; // no period match

    /* Configure IC1 and IC2 as a single 32-bit input capture module. This
    ** module will be used to capture a timestamp when a falling edge or rising
    ** edge is detected on the PWM input pin. Note: this module must be enabled
    ** prior to Timer 2, which provides the clock source.
    */
    IC1CON1bits.ICSIDL = 0; // continue to run during idle
    IC1CON1bits.ICTSEL = 0b001; // T2CLK is clock source
    IC1CON1bits.ICI = 0b00; // interrupt on every capture event
    IC1CON2bits.IC32 = 1; // cascade mode enabled to form 32-bit timer
    IC1CON2bits.ICTRIG = 0; // sync ic module to another timer
    IC1CON2bits.SYNCSEL = 0b00000; // no sync or trigger source for this module

    IC2CON1bits.ICSIDL = 0; // continue to run during idle
    IC2CON1bits.ICTSEL = 0b001; // T2CLK is clock source
    IC2CON1bits.ICI = 0b00; // interrupt on every capture event
    IC2CON2bits.IC32 = 1; // cascade mode enabled to form 32-bit timer
    IC2CON2bits.ICTRIG = 0; // sync ic module to another timer
    IC2CON2bits.SYNCSEL = 0b00000; // no sync or trigger source for this module

    IPC1bits.IC2IP = 7; // set interrupt priority
    IFS0bits.IC2IF = 0; // clear interrupt flag
    IEC0bits.IC2IE = 1; // enable interrupt

    /* Map pins for input capture module.
    */
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR7bits.IC1R = ppsServoIn;
    RPINR7bits.IC2R = ppsServoIn;
    __builtin_write_OSCCONL(OSCCON | (1<<6));
}

/* ------------------------------------------------------------ */
/***	ServoEnable
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
**      This function configures enables the input capture modules (IC1 and IC2)
**      to capture input on every rising and falling edge and enables the
**      timer (Timer 2) that provides the time base for measuring the edges.
**
**  Notes:
**      This function does NOT configure the input capture modules or the
**      timers. You MUST call ServoInit() prior to calling this function.
*/
void
ServoEnable() {

    /* Enable 32-bit interrupt capture module and start measuring any pulse
    ** width that may be applied on the PWM input pin.
    */
    IC2CON1bits.ICM = 0b001; // capture every rising and falling edge, enable module
    IC1CON1bits.ICM = 0b001; // capture every rising and falling edge, enable module
    T2CONbits.TON = 1; // start Timer 2, which provides the clock source for IC
}

/* ------------------------------------------------------------ */
/***	ServoDisable
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
**      This function disables the input capture modules (IC1 and IC2) that are
**      used for measuring the servo input pulse width. It also disables the
**      clock source (Timer 2), resets its count, clears the IC2 interrupt flag,
**      and aborts any ongoing servo calibration.
*/
void
ServoDisable() {

    WORD    cnt;

    /* Turn off the input capture module.
    */
    IC1CON1bits.ICM = 0b000;
    IC2CON1bits.ICM = 0b000;

    /* Flush anything that's in the input capture buffer so that we don't
    ** read old data later.
    */
    while ( 1 == IC2CON1bits.ICBNE ) {
        cnt = IC1BUF;
        cnt = IC2BUF;
    }

    /* Disable the timer used as the clock source for the input capture
    ** module since we aren't presently using the module.
    */
    T2CONbits.TON = 0; // stop Timer 2, which provides the clock source for IC
    TMR2 = 0; // clear timer count register

    /* Clear the interrupt flag so that we don't process any pending servo
    ** interrupt.
    */
    IFS0bits.IC2IF = 0;

    /* Abort any servo calibration proceedure that may have been taking place.
    */
    ServoAbortCalibration();
}

/* ------------------------------------------------------------ */
/***	ServoRestoreCalDefaults
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
**      This function replaces the running calibration constants with the
**      factory default calibration values and writes them to the flash.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
ServoRestoreCalDefaults() {

    /* Update the calibration constants.
    */
    servocalCur.cntServoMin = cntServoMinDefault;
    servocalCur.cntServoMax = cntServoMaxDefault;
    servocalCur.cntServoNeutralMin = cntServoNeutralMinDefault;
    servocalCur.cntServoNeutralMax = cntServoNeutralMaxDefault;

    /* Update the calibration constants in configuration memory and write the
    ** configuration to flash. Please note that this function may take up to
    ** 54ms to execute and that the CPU is stalled during this time.
    */
    CfgSetServoCal(&servocalCur);

    /* Skip any control interrupt that may be pending. This is necesary because
    ** if there is also an ADC interrupt pending then the ADC interrupt will
    ** be processed and the channel being sampled will be changed. If we allow
    ** a pending control interrupt to execute immediately following the ADC
    ** interrupt then there is a high probability of starting an ADC conversion
    ** prior to the required sampling time being met, which will result in an
    ** incorrect conversion.
    */
    CtrlrSkipNextInterrupt();
}

/* ------------------------------------------------------------ */
/***	ServoStartCalibration
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
**      This function sets up the calibration variables, places the servo
**      interface in calibration mode, sets the output voltage to 0, and
**      informs the LED state machine that calibration is going to take place.
**
**  Notes:
**      The calibration state will not be entered if there is already an active
**      calibration proceedure, nor will it be entered if there is no active
**      servo link.
**
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
ServoStartCalibration() {

    /* Do not start a calibration proceedure if we are already calibrating
    ** or if we don't have an established servo link.
    ** M00TOD: consider checking for controller faults.
    */
    if (( calstIdle != calstServo ) || ( linktServo != CtrlrGetLinkType() )) {
        return;
    }

    /* Set the min and max pulse widths appropriately before entering the
    ** calibraiton state.
    */
    cntServoMinNew = 0xFFFF;
    cntServoMaxNew = 0;
    cntServoNeutralNew = cntServoInvalid;
    calstServo = calstActive;

    /* Set the output voltage to the neutral voltage so that the motors stop
    ** spinning.
    */
    CmdForceNeutral();

    /* Tell the LED state machine that we are going to begin calibration so that
    ** it will display the appropriate status.
    */
    LedCalibrationStart();
}

/* ------------------------------------------------------------ */
/***	ServoEndCalibration
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
**      This function change the calibration state to calstAdjust, verifies
**      that the new calibration constants are valid, replaces the running
**      constants with the new calibration constants, writes them to flash,
**      changes the calibration state to calstIdle, and then informs the LED
**      state machine that calibration was successful.
**
**      If the calibration constants aren't valid then the new constants are
**      discard, the calibration state is changed to calstIdle, and the LED
**      state machine is informed that calibraiton failed.
**
**  Notes:
**      If the current calibration state isn't calstActive or the current link
**      type isn't linktServo, then this function doesn't nothing and returns
**      immediately.
**
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
ServoEndCalibration() {

    /* Make sure that calibration was actually active, and that it wasn't
    ** aborted prior to this function being called as a result of a lost link
    ** or a fault.
    */
    if (( calstActive != calstServo ) || ( linktServo != CtrlrGetLinkType() )) {
        return;
    }

    /* Stop updating the calibration variables.
    */
    calstServo = calstAdjust;

    /* Check to make sure that a neutral pulse width was measured.
    */
    if ( cntServoInvalid == cntServoNeutralNew ) {
        goto lErrorExit;
    }
    
    /* Check to make sure that the minimum and maximum pulse widths are within
    ** the allowed range.
    */
    if (( cntServoMinAbsolute > cntServoMinNew ) || 
        ( cntServoMaxAbsolute < cntServoMaxNew )) {
        goto lErrorExit;
    }
    
    /* Check to make sure that the minimum pulse width is less than the neutral
    ** pulse width and that the neutral pulse width is less than the maximum
    ** pulse width.
    */
    if (( cntServoMinNew >= cntServoNeutralNew ) || 
        ( cntServoMaxNew <= cntServoNeutralNew )) {
        goto lErrorExit;
    }

    /* Check to make sure that the minimum and maximum pulse widths meet the
    ** minimum range requirements for mapping to both positive and negative
    ** voltages.
    */
    if (( cntServoRangehMin > (cntServoNeutralNew - cntServoMinNew) ) ||
        ( cntServoRangehMin > (cntServoMaxNew - cntServoNeutralNew) )) {
        goto lErrorExit;
    }

    /* Update the calibration constants.
    */
    servocalCur.cntServoMin = cntServoMinNew;
    servocalCur.cntServoMax = cntServoMaxNew;
    servocalCur.cntServoNeutralMin = cntServoNeutralNew - (((cntServoNeutralNew - cntServoMinNew) * 4) / 100);
    servocalCur.cntServoNeutralMax = cntServoNeutralNew + (((cntServoMaxNew - cntServoNeutralNew) * 4) / 100);

    /* Update the calibration constants in configuration memory and write the
    ** configuration to flash. Please note that this function may take up to
    ** 54ms to execute and that the CPU is stalled during this time.
    */
    CfgSetServoCal(&servocalCur);

    /* Skip any control interrupt that may be pending. This is necesary because
    ** if there is also an ADC interrupt pending then the ADC interrupt will
    ** be processed and the channel being sampled will be changed. If we allow
    ** a pending control interrupt to execute immediately following the ADC
    ** interrupt then there is a high probability of starting an ADC conversion
    ** prior to the required sampling time being met, which will result in an
    ** incorrect conversion.
    */
    CtrlrSkipNextInterrupt();

    /* Exit calibration mode.
    */
    calstServo = calstIdle;

    /* Tell the LED state machine to display the status of the now completed
    ** calibration proceedure.
    */
    LedCalibrationSuccess();

    return;

lErrorExit:

    calstServo = calstIdle;
    LedCalibrationFailed();
}

/* ------------------------------------------------------------ */
/***	ServoAbortCalibration
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
**      This function change the calibration state to calstIdle, which will
**      immediately abort any calibration proceedure that is in progress.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
void
ServoAbortCalibration() {

    calstServo = calstIdle;

    /* Tell the LED state machine that calibration failed. This is necessary
    ** because it's possible that this function will be called while the user
    ** is performing calibration. Since we set the calst to calstIdle the
    ** ServoEndCalibration() function will no longer do anything when the user
    ** releases the CAL button. As a result, neither the calibration failed or
    ** succeeded flags will be set and the LED state machine will be stuck in
    ** the calibration state forever. Calling LedCalibrationFailed() will allow
    ** us to avoid the LED state machine getting stuck in the calibration state.
    */
    LedCalibrationFailed();
}

/* ------------------------------------------------------------ */
/***	ServoGetCalibrationState
**
**  Parameters:
**      none
**
**  Return Value:
**      current calibration state
**
**  Errors:
**      none
**
**  Description:
**      This function returns the current calibration state associated with the
**      servo interface.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
WORD
ServoGetCalibrationState() {

    return calstServo;
}

/* ------------------------------------------------------------ */
/***	ServoPwToVoltage
**
**  Parameters:
**      cntServoPw  - pulse width measured for the servo input
**
**  Return Value:
**      voltage corresponding to the servo pulse width, 32767 for the maximum
**      pulse width, -32768 for the minimum pulse width, 0 for anything that
**      falls within the neutral region
**
**  Errors:
**      none
**
**  Description:
**      This function maps the specified servo pulse width to a voltage. If the
**      pulse width is greater than cntServoNeutralMax then the pulse width will
**      be converted to a positive voltage between 1 and 32767. If the pulse
**      width is less than cntServoNeutralMin then the pulse width will be
**      converted to a negative voltage between -1 and -32768. If the pulse
**      width falls within cntServoNeutralMin and cntServoNeutralMax then the
**      pulse width will be converted to a voltage of 0.
**
**  Notes:
**      The current implementation assumes that the servo interrupt handler
**      treats any pulse width that is less than cntServoMinAbsolute or greater
**      than cntServoMaxAbsolute as an invalid pulse width and therefore does
**      not call this function.
**
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int16_t
ServoPwToVoltage(WORD cntServoPw) {

    int16_t vltgPw;
    
    /* Limit the pulse width to be within the range that we are currently
    ** calibrated for with the assumption being that the servo interrupt handler
    ** will treat any pulse width that's between cntServoMinAbsolute and
    ** cntServoMaxAbsolute as a valid pulse width regardless of our current
    ** calibration constants.
    */
    if ( servocalCur.cntServoMax  < cntServoPw ) {
        cntServoPw = servocalCur.cntServoMax;
    }
    else if ( servocalCur.cntServoMin > cntServoPw ) {
        cntServoPw = servocalCur.cntServoMin;
    }

    /* Convert the pulse width to a voltage.
    */
    if ( servocalCur.cntServoNeutralMax  < cntServoPw ) {
        vltgPw = (((DWORD)(cntServoPw - servocalCur.cntServoNeutralMax)) * 32767) / (servocalCur.cntServoMax - servocalCur.cntServoNeutralMax);
    }
    else if ( servocalCur.cntServoNeutralMin > cntServoPw ) {
        vltgPw = 0 - ((((DWORD)(servocalCur.cntServoNeutralMin - cntServoPw)) * 32768) / (servocalCur.cntServoNeutralMin  - servocalCur.cntServoMin ));
    }
    else {
        vltgPw = 0;
    }

    return vltgPw;
}

/******************************************************************************/