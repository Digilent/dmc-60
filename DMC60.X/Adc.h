/************************************************************************/
/*									*/
/*  Adc.h - Motor Controller ADC Functions                              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains declarations of routines that can be used to	*/
/*  configure the ADC for monitoring the current, temperature, and bus  */
/*  voltage of the motor controller. It also includes the decalaration  */
/*  of routines that return the current, temperature, and bus voltage.  */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  05/06/2016 (MichaelA): created			                */
/*  05/12/2016 (MichaelA): added function to return the temperature as  */
/*      a signed 8.8 fixed point number                                 */
/*  09/02/2016 (MichaelA): added functions to return the current and    */
/*      bus voltage as 8.8 fixed point numbers                          */
/*									*/
/************************************************************************/

#if !defined(_ADC_INC)
#define	_ADC_INC

#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*                  Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* Define macro used to start an ADC conversion.
*/
#define AdcStartConversion()    (AD1CON1bits.SAMP = 0)

/* ------------------------------------------------------------ */
/*                  General Type Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Variable Declarations			*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Procedure Declarations			*/
/* ------------------------------------------------------------ */

void    AdcInterruptHandler();
void    AdcInit();

BOOL    AdcCalibrationComplete();
int32_t AdcGetCurrent();
int16_t AdcGetCurrent16();
int32_t AdcGetTemperature();
int16_t AdcGetTemperature16();
int32_t AdcGetVbus();
int16_t AdcGetVbus16();

/* ------------------------------------------------------------ */


#endif
