/************************************************************************/
/*									*/
/*  Servo.h - Motor Controller Servo (PWM) Input Interface Functions    */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that are uesd to      */
/*  implement the servo (pwm) input interface. This includes routines   */
/*  for setting the calibration state, measuring the pulse width, and   */
/*  for mapping an input pulse width to an output voltage.              */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  04/27/2016 (MichaelA): created			                */
/*  05/26/2016 (MichaelA): redefined the default neutral zone values    */
/*  08/04/2016 (MichaelA): changed MinAbsolute and MaxAbsolute pulse    */
/*      width definitions to allow for some slop in the measurement     */
/*  09/02/2016 (MichaelA): added ServoDisable() to allow servo pulse    */
/*      width measurement to be disabled when CAN bus is detected       */
/*  11/02/2016 (MichaelA): added ServoRestoreCalDefaults() to allow     */
/*      the calibration constants to be restored to their default state */
/*									*/
/************************************************************************/

#if !defined(_SERVO_INC)
#define	_SERVO_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define servo interface calibration states.
*/
#define calstIdle               0 // no calibration in progress
#define calstActive             1 // calibration in progress
#define calstAdjust             2 // calibratoin completed, verifying constants

/* Define constants for use in measuring the Servo Pulse Width. Please note that
** the Neutral deadband is defined as 4% in order to be compatible with systems
** that also utilize the Talon SRX, which employs a 4% neutral deadband.
*/
#define cntServoRangehMin           200     // 0.2ms
#define cntServoMinAbsolute         595     // 0.6ms + 5us of slop
#define cntServoMaxAbsolute         2405    // 2.4ms + 5us of slop
#define cntServoMinDefault          1000    // 1ms
#define cntServoMaxDefault          2000    // 2ms
#define cntServoNeutralDefault      1500    // 1.5ms
#define cntServoNeutralMinDefault   (cntServoNeutralDefault - (((cntServoNeutralDefault - cntServoMinDefault) * 4) / 100))
#define cntServoNeutralMaxDefault   (cntServoNeutralDefault + (((cntServoMaxDefault - cntServoNeutralDefault) * 4) / 100))
#define cntServoInvalid             0

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */

typedef struct {
    WORD    cntServoMin;
    WORD    cntServoMax;
    WORD    cntServoNeutralMin;
    WORD    cntServoNeutralMax;
} SERVOCAL;

/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    ServoInterruptHandler();

void    ServoInit();
void    ServoEnable();
void    ServoDisable();

void    ServoRestoreCalDefaults();
void    ServoStartCalibration();
void    ServoEndCalibration();
void    ServoAbortCalibration();
WORD    ServoGetCalibrationState();

int16_t ServoPwToVoltage(WORD cntServoPw);

/* ------------------------------------------------------------ */


#endif