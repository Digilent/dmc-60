/************************************************************************/
/*									*/
/*  Adc.c - Motor Controller ADC Functions                              */
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander					*/
/*  Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module contains definitions of routines that can be used to	*/
/*  configure the ADC for monitoring the current, temperature, and bus  */
/*  voltage of the motor controller. It also includes the definitions   */
/*  of routines that return the current, temperature, and bus voltage.  */
/*									*/
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  05/06/2016 (MichaelA): created			                */
/*  05/10/2016 (MichaelA): implemented basic over current detection     */
/*  05/11/2016 (MichaelA): implemented under voltage and over           */
/*      temperature detection                                           */
/*  05/12/2016 (MichaelA): added function to return the temperature as  */
/*      a signed 8.8 fixed point number                                 */
/*  06/24/2016 (MichaelA): reduced tmpAmbOT to 100 degrees C            */
/*  07/18/2016 (MichaelA): increased the amount of time it takes for an */
/*      undervoltage condition to cause an undervoltage fault from 1    */
/*      second to 5 seconds                                             */
/*  08/02/2016 (MichaelA): modified AdcInterruptHandler so that after   */
/*      an undervoltage fault occurs an undervoltage fault will         */
/*      continue to be signaled until the bus voltage exceeds the       */
/*      undervoltage threshold. This is necessary to prevent the output */
/*      from being re-enabled before the undervoltage condition has     */
/*      been corrected                                                  */
/*  08/19/2016 (MichaelA): modified AdcInterruptHandler and AdcInit     */
/*      such that current monitoring can be conditionally enabled by a  */
/*      compile time constant                                           */
/*  09/02/2016 (MichaelA): added functions to return the current and    */
/*      bus voltage as 8.8 fixed point numbers                          */
/*  11/01/2016 (MichaelA): changed the under voltage threshold to 5.75V */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Ctrlr.h"
#include "Adc.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define ID's to keep track of which ADC channel is presently being sampled.
*/
#define idsmpVmon1      0
#define idsmpImon1      1
#define idsmpTmon1      2
#define idsmpUserAN1    3

/* Define the maximum number of samples that can be stored in a buffer.
*/
#define csmpVmon1Max    512
#define csmpImon1Max    csmpVmon1Max
#define csmpTmon1Max    csmpImon1Max

#define csmpCalMax      100

/* Define the under voltage threshold, as well as the number of consecutive
** voltage readings required to signal the undervoltage condition.
** Please note that the under voltage threshold is defined as a signed 16.16
** fixed point value, which specifies the voltage in Volts.
*/
#define vltgVbusUV      0x0005C000  // 5.75 volts
#define csmpVbusUVMax   10000       // 5 seconds

/* Define the forward and reverse overcurrent thresholds, as well as the number
** of consecutive current readings required to signal the overcurrent condition.
** Please note that the overcurrent thresholds are defined as signed 16.16 fixed
** point values.
*/
#define crntFwdOC1      0x00500000  // 80.0 amps
#define crntFwdOC2      0x00730000  // 115.0 amps
#define crntRevOC1      0xFFB00000  // -80.0 amps
#define crntRevOC2      0xFF8D0000  // -115.0 amps

#define csmpOC1Max      8000        // 4 seconds
#if defined(DEAD)
#define csmpOC2Max      20          // 10 milliseconds
#else
#define csmpOC2Max      600         // 300 milliseconds
#endif

#define crntFwdHighGMax 0x00320000  // 50.0 amps
#define crntRevHighGMax 0xFFCE0000  // -50.0 amps

#define crntFwdLowGMin  0x00280000  // 40.0 amps
#define crntRevLowGMin  0xFFD80000  // -40.0amps
#define csmpLowGainMax  100         // 50 milliseconds

/* Define the overtemperature threshold, as well as the number of consecutive
** temperature readings required to signal the overtemperature condition.
** Please note that the overtemperature threshold is defined as a signed 16.16
** fixed point value, which specifies a temperature in degrees C.
*/
#if defined(DEAD)
#define tmpAmbOT        0x00640000  // 100.0 degrees C
#define csmpAmbOTMax    4           // 2 milliseconds
#else
#define tmpAmbOT        0x00690000  // 105.0 degrees C
#define csmpAmbOTMax    4           // 2 milliseconds
#endif

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

/* Declare variables used for sampling analog inputs.
*/
static volatile WORD    smpVmon1Cur;
static volatile WORD    smpImon1Cur;
static volatile WORD    smpTmon1Cur;
static volatile WORD    smpUserAN1Cur;
static volatile WORD    idsmpCur;

/* Declare variables sued to calculate and keep track of the input
** supply voltage (VBUS).
*/
static volatile int32_t vltgVbus = 0;
static volatile WORD    csmpVbusUV = 0;

/* Declare variables used to calculate and keep track of the motor
** winding current and for calculating the motor winding current,
** and for keeping track of the number of measurements for which
** the motor winding current has exceeded the pre-defined current
** limits.
*/
static volatile DWORD   smpCal = 0;
static volatile WORD    csmpCal = 0;
static volatile WORD    smpZeroCurrent = 0;
static volatile int32_t crntMotor = 0;
static volatile WORD    csmpOC1 = 0;
static volatile WORD    csmpOC2 = 0;
static volatile WORD    csmpLowG = 0;

/* Declare variables used to calculate and keep track of the ambient
** temperature inside of the motor controller case.
*/
static volatile int32_t tmpAmbient = 0;
static volatile WORD    csmpAmbOT = 0;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	AdcInterruptHandler
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
**      This function is called each time an ADC interrupt is signaled as the
**      result of an ADC conversion completing. It stores the raw sample
**      associated with the currently select analog channel in the appropriate
**      variable, performs any required calculations, and checks for fault
**      conditions. Any fault that is detected will be signaled so that the
**      control loop can perform the appropriate actions.
**
**  Notes:
**      This function does NOT clear the interrupt flag. It is up to the caller
**      to clear the interrupt flag associated with the interrupt service
**      routine that called this function.
*/
void
AdcInterruptHandler() {

    WORD            smpTmp;
    static BOOL     fUVFault = fFalse;

    /* Read the converted value from the buffer.
    */
    smpTmp = ADC1BUF0;

    /* Store the current conversion result in the appropriate variable and
    ** change the input mux to the next channel that's to be sampled.
    */
    switch ( idsmpCur ) {
        case idsmpVmon1:
            smpVmon1Cur = smpTmp;
            AD1CHS0bits.CH0SA = adcImon1;
            idsmpCur = idsmpImon1;

            /* Calculate the current input voltage (VBUS) and check to see if an
            ** undervoltage fault needs to be signaled.
            */
            vltgVbus = (((int32_t)smpTmp) * 0x000001E0);
            if ( vltgVbusUV > vltgVbus ) {
                csmpVbusUV++;
                if ( csmpVbusUVMax <= csmpVbusUV ) {
                    CtrlrSignalFault(fltUnderVoltage);
                    csmpVbusUV = 0;
                    fUVFault = fTrue;
                }
                else if ( fUVFault ) {
                    CtrlrSignalFault(fltUnderVoltage);
                }
            }
            else {
                csmpVbusUV = 0;
                fUVFault = fFalse;
            }

            break;
            
        case idsmpImon1:
            smpImon1Cur = smpTmp;
            AD1CHS0bits.CH0SA = adcTmon1;
            idsmpCur = idsmpTmon1;

#if defined(MONITOR_CURRENT)
            /* Check to see if calibration has been completed and then take the
            ** appropriate action.
            */
            if ( 0 < smpZeroCurrent ) {
                /* Calculate the motor winding current.
                */
                if ( latGainSel & (1 << bnGainSel) ) {
                    /* Calculate current using the high gain (50V/V) constant.
                    ** Reduce gain immediately if the present reading exceeds
                    ** a predefined threshold.
                    */
                    crntMotor = (((int32_t)smpTmp - smpZeroCurrent) * 0x00000780);
                    if (( crntFwdHighGMax <= crntMotor ) ||
                        ( crntRevHighGMax >= crntMotor )) {
                        latGainSel &= ~(1 << bnGainSel);
                    }
                }
                else {
                    /* Calculate current using the low gain (25V/V) constant.
                    ** Increase the gain after a predefined number of
                    ** consecutive samples falling within a predefined
                    ** threshold.
                    */
                    crntMotor = (((int32_t)smpTmp - smpZeroCurrent) * 0x00000F00);
                    if (( crntFwdLowGMin > crntMotor ) &&
                        ( crntRevLowGMin < crntMotor )) {
                        csmpLowG++;
                        if ( csmpLowGainMax <= csmpLowG ) {
                            latGainSel |= (1 << bnGainSel);
                            csmpLowG = 0;
                        }
                    }
                    else {
                        csmpLowG = 0;
                    }
                }

                /* Check for overcurrent conditions and signal a fault when
                ** necessary.
                */
                if (( 0 < crntMotor ) && ( crntFwdOC2 <= crntMotor )) {
                    csmpOC1++;
                    csmpOC2++;
                    if (( csmpOC2Max <= csmpOC2 ) || ( csmpOC1Max <= csmpOC1 )) {
                        CtrlrSignalFault(fltOverCurrent);
                        csmpOC1 = 0;
                        csmpOC2 = 0;
                    }
                }
                else if (( 0 < crntMotor ) && ( crntFwdOC1 <= crntMotor )) {
                    csmpOC1++;
                    csmpOC2 = 0;
                    if ( csmpOC1Max <= csmpOC1 ) {
                        CtrlrSignalFault(fltOverCurrent);
                        csmpOC1 = 0;
                    }
                }
                else if (( 0 > crntMotor ) && ( crntRevOC2 >= crntMotor )) {
                    csmpOC1++;
                    csmpOC2++;
                    if (( csmpOC2Max <= csmpOC2 ) || ( csmpOC1Max <= csmpOC1 )) {
                        CtrlrSignalFault(fltOverCurrent);
                        csmpOC1 = 0;
                        csmpOC2 = 0;
                    }
                }
                else if (( 0 > crntMotor ) && ( crntRevOC1 >= crntMotor )) {
                    csmpOC1++;
                    csmpOC2 = 0;
                    if ( csmpOC1Max <= csmpOC1 ) {
                        CtrlrSignalFault(fltOverCurrent);
                        csmpOC1 = 0;
                    }
                }
                else {
                    csmpOC1 = 0;
                    csmpOC2 = 0;
                }
            }
            else {
                /* We haven't completed the calibration process.
                */
                smpCal += smpTmp;
                csmpCal++;
                if ( csmpCalMax == csmpCal ) {
                    smpZeroCurrent = ( smpCal / (DWORD)csmpCalMax );
                    if ( (csmpCalMax >> 1)  <=  (smpCal % csmpCalMax) ) {
                        smpZeroCurrent++;
                    }
                }
            }
#endif

            break;

        case idsmpTmon1:
            smpTmon1Cur = smpTmp;
            AD1CHS0bits.CH0SA = adcVmon1;
            idsmpCur = idsmpVmon1;
            
            /* Calculate the temperature in degrees C and then check to see if
            ** an over temperature fault needs to be signaled.
            */
            tmpAmbient = (((int32_t)smpTmp) * 0x000012C0) - 0x00320000;
            if ( tmpAmbOT <= tmpAmbient ) {
                csmpAmbOT++;
                if ( csmpAmbOTMax <= csmpAmbOT ) {
                    CtrlrSignalFault(fltOverTemp);
                    csmpAmbOT = 0;
                }
            }
            else {
                csmpAmbOT = 0;
            }

            break;

        case idsmpUserAN1:
        default:
            AD1CHS0bits.CH0SA = adcVmon1;
            idsmpCur = idsmpVmon1;
            break;
    }

}

/* ------------------------------------------------------------ */
/***	AdcInit
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
**      This function initializes the ADC. It disables the ADC module, sets the
**      default values for all control variables, configures the appropriate
**      input pins as analog inputs, configures the ADC for automatic sample
**      with manual conversion mode, enables the ADC interrupt, and starts the
**      ADC module.
**
**  Notes:
**      The ADC requires approximately 20us between the time that it's enabled
**      and the time that samples can be considered valid. This function does
**      NOT implement the necessary delay. It is the responsibility of the
**      caller to ensure that no conversions are performed until after a minimum
**      of 20us has elapsed.
*/
void
AdcInit() {

    /* Disable the ADC module.
    */
    AD1CON1bits.ADON = 0;

    /* Set default values for variables used to keep track of the analog inputs.
    */
    smpVmon1Cur = 0;
    smpImon1Cur = 0;
    smpTmon1Cur = 0;
    smpUserAN1Cur = 0;
    idsmpCur = idsmpVmon1;

    /* Set the default values for the variables that are used to calculate and
    ** keep track of the input voltage (VBUS) of the motor controller.
    */
    vltgVbus = 0;
    csmpVbusUV = 0;

    /* Set the default values for the variables that are used to calculate and
    ** keep track of the motor winding current.
    */
    smpCal = 0;
    csmpCal = 0;
    crntMotor = 0;
    csmpOC1 = 0;
    csmpOC2 = 0;
    csmpLowG = 0;

    /* Set the sample value corresponding to zero-current based. How this value
    ** gets set depends on whether or not the current sense amplifier is
    ** installed. If the current sense amplifier is installed then the initial
    ** value should be set to 0, which will cause calibration to take place
    ** after initial power-on. If the current sense amplifier isn't installed
    ** then the initial value should be set > 0 so that the control loop doesn't
    ** wait for calibration to take place, as it will never complete.
    */
#if defined(MONITOR_CURRENT)
    smpZeroCurrent = 0;
#else
    smpZeroCurrent = 2048;
#endif

    /* Set the default values for the variables that are used to calculate and
    ** keep track of the ambient temperature inside of the motor controller
    ** case.
    */
    tmpAmbient = 0;
    csmpAmbOT = 0;

    /* Configure the gain select pin for the current shunt amplifier as an
    ** output and set the default gain to 50V/V.
    */
    latGainSel |= (1 << bnGainSel);
    trsGainSel &= ~(1 << bnGainSel);

    /* Configure Vmon1, Imon1, Tmon1, and UserAN1 as inputs and enable analog
    ** input on those pins.
    */
    trsVmon1 |= (1 << bnVmon1);
    trsImon1 |= (1 << bnImon1);
    trsTmon1 |= (1 << bnTmon1);
    trsUserAN1 |= (1 << bnUserAN1);
    ansVmon1 |= (1 << bnVmon1);
    ansImon1 |= (1 << bnImon1);
    ansTmon1 |= (1 << bnTmon1);
    ansUserAN1 |= (1 << bnUserAN1);

    /* Configure the ADC module.
    */
    AD1CON1bits.ADSIDL = 0; // continue in idle mode
    AD1CON1bits.AD12B = 1;  // enable 12-bit mode
    AD1CON1bits.FORM = 0b00; // integer output, no sign
    AD1CON1bits.SSRCG = 0; // use group 0 sample source
    AD1CON1bits.SSRC = 0b000; // manual sample mode
    AD1CON1bits.SIMSAM = 0; // simultaneous sample not available in 12-bit mode
    AD1CON1bits.ASAM = 1; // enable automatic sampling

    AD1CON2bits.VCFG = 0b001; // use external VREF+ and AVSS for VREF-
    AD1CON2bits.CSCNA = 0; // do not scan inputs
    AD1CON2bits.CHPS = 0b00; // convert CH0
    AD1CON2bits.SMPI = 0b0000; // interrupt after every conversion
    AD1CON2bits.BUFM = 0; // always fill buffer from start address
    AD1CON2bits.ALTS = 0; // always use channel input select for sample MUXA

    AD1CON3bits.ADRC = 0; // dervice ADC clock from system clock
    AD1CON3bits.SAMC = 4; // sample time = 4TAD when auto convert is enabled
    AD1CON3bits.ADCS = 7; // TAD = TCY/8 = 125ns (8MHz)

    AD1CON4bits.ADDMAEN = 0; // DMA is not enabled

    AD1CHS0bits.CH0NB = 0; // channel 0 negative input is VREFL
    AD1CHS0bits.CH0SB = adcVmon1; // select VMON1 channel as positive input
    AD1CHS0bits.CH0NA = 0; // select VREFL as negative input
    AD1CHS0bits.CH0SA = adcVmon1; // select VMON1 channel as positive input

    IPC3bits.AD1IP = 7; // set interrupt priority
    IFS0bits.AD1IF = 0; // clear interrupt flag
    IEC0bits.AD1IE = 1; // enable interrutps
    AD1CON1bits.ADON = 1; // turn on the ADC
}

/* ------------------------------------------------------------ */
/***	AdcCalibrationComplete
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if calibration has completed, fFalse otherwise
**
**  Errors:
**      none
**
**  Description:
**      This function returns the status of the ADC calibration proceedure. If
**      calibration has successfully completed then fTrue will be returned. If
**      calibration is still underway then fFalse is returned.
**
**  Notes:
**      During calibration it is assumed that the bridge winding current is
**      zero. During this time the ADC interrupt handler will take some number
**      of samples and average them to determine the sample value corresponding
**      to zero current flow. This value is later used to calculate the winding
**      current.
*/
BOOL
AdcCalibrationComplete() {

    return ( 0 < smpZeroCurrent ) ? fTrue : fFalse;
}

/* ------------------------------------------------------------ */
/***	AdcGetCurrent
**
**  Parameters:
**      none
**
**  Return Value:
**      motor winding current
**
**  Errors:
**      none
**
**  Description:
**      This function returns the motor winding current as a signed 16.16 fixed
**      point value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int32_t
AdcGetCurrent() {

    return crntMotor;
}

/* ------------------------------------------------------------ */
/***	AdcGetCurrent16
**
**  Parameters:
**      none
**
**  Return Value:
**      motor winding current
**
**  Errors:
**      none
**
**  Description:
**      This function returns the motor winding current as a signed 8.8 fixed
**      point value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int16_t 
AdcGetCurrent16() {
    
    return (crntMotor >> 8);
}

/* ------------------------------------------------------------ */
/***	AdcGetTemperature
**
**  Parameters:
**      none
**
**  Return Value:
**      motor winding current
**
**  Errors:
**      none
**
**  Description:
**      This function returns the ambient temperature inside of the motor
**      controller case (in degrees Celcius) as a signed 16.16 fixed point
**      value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int32_t
AdcGetTemperature() {

    return tmpAmbient;
}

/* ------------------------------------------------------------ */
/***	AdcGetTemperature16
**
**  Parameters:
**      none
**
**  Return Value:
**      ambient temperature in degrees Celcius
**
**  Errors:
**      none
**
**  Description:
**      This function returns the ambient temperature inside of the motor
**      controller case (in degrees Celcius) as a signed 8.8 fixed point
**      value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int16_t
AdcGetTemperature16() {

    return (tmpAmbient >> 8);
}

/* ------------------------------------------------------------ */
/***	AdcGetVbus
**
**  Parameters:
**      none
**
**  Return Value:
**      input voltate (VBUS) to the motor controller
**
**  Errors:
**      none
**
**  Description:
**      This function returns the input voltage (VBUS in volts) being applied
**      to the VIN pin of the motor controller as a igned 16.16 fixed point
**      value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int32_t AdcGetVbus() {

    return vltgVbus;
}

/* ------------------------------------------------------------ */
/***	AdcGetVbus16
**
**  Parameters:
**      none
**
**  Return Value:
**      input voltate (VBUS) to the motor controller
**
**  Errors:
**      none
**
**  Description:
**      This function returns the input voltage (VBUS in volts) being applied
**      to the VIN pin of the motor controller as a igned 8.8 fixed point
**      value.
**
**  Notes:
**      The current implementation assumes that this function will ONLY be
**      called from ISR context and that all interrupts have the same user
**      priority, which means that no nesting will occur.
*/
int16_t AdcGetVbus16() {

    return (vltgVbus >> 8);
}

/******************************************************************************/
