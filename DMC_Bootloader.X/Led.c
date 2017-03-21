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
/*  09/13/2016 (MichaelA): created			                */
/*  10/11/2016 (MichaelA): added LedDispProgFlash() to allow CAN bus to */
/*      tell the LED state machine that the flash has been written      */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include <string.h>
#include "DMC.h"
#include "stdtypes.h"
#include "Can.h"
#include "Led.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define the states that the LED state machine can be in.
*/
#define ledstInit           0
#define ledstLinkPresent    1
#define ledstNoLink         2
#define ledstDelay          3

/* Define the maximum number of LED on/off states.
*/
#define cledoostMax     4

/* Define the LED duty cycle for displaying status in various states.
*/
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
    24,
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

    if ( ledflgs.fDispProgFlash ) {
        ledflgs.fDispProgFlash = 0;
        if ( ledstDelay != ledstCur ) {
            LedStrobeOpposite(dtcLedNoLink, dtcLedNoLink >> 1, 0, 100, 0, fFalse);
            ctickDelay = LedMsToTicks(1000);
            ledstCur = ledstDelay;
        }
        else {
            ctickDelay += LedMsToTicks(200);
        }
    }

    /* Perform an appropriate action based on the current state of the LED state
    ** machine.
    */
    switch ( ledstCur ) {
        case ledstInit:
            if ( CanIsLinkPresent() ) {
                LedStrobeOpposite(dtcLedNoLink, dtcLedNoLink >> 1, 0, 500, 0, fFalse);
                ledstCur = ledstLinkPresent;
            }
            else {
                LedStrobeOpposite(dtcLedNoLink, 0, 0, 500, 0, fFalse);
                ledstCur = ledstNoLink;
            }
            break;

        case ledstLinkPresent:
            break;

        case ledstNoLink:
            if ( CanIsLinkPresent() ) {
                LedStrobeOpposite(dtcLedNoLink, dtcLedNoLink >> 1, 0, 500, 0, fFalse);
                ledstCur = ledstLinkPresent;
            }
            break;

        case ledstDelay:
            ctickDelay--;
            if ( 0 == ctickDelay ) {
                if ( CanIsLinkPresent() ) {
                    LedStrobeOpposite(dtcLedNoLink, dtcLedNoLink >> 1, 0, 500, 0, fFalse);
                    ledstCur = ledstLinkPresent;
                }
                else {
                    LedStrobeOpposite(dtcLedNoLink, 0, 0, 500, 0, fFalse);
                    ledstCur = ledstNoLink;
                }
            }
            break;
    }

    /* Check to see if we've lost the CAN link.
    */
    if (( ! CanIsLinkPresent() ) &&
        ( ledstNoLink != ledstCur ) &&
        ( ledstDelay != ledstCur )) {
        /* We've lost the CAN link.
        */
        LedStrobeOpposite(dtcLedNoLink, 0, 0, 500, 0, fFalse);
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
    latLedBC &= ~(1 << bnLedBC);
    trsLedBC &= ~(1 << bnLedBC);
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
/***	LedTerm
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
**      This function performs any cleanup necessary to turn off the onboard
**      LEDs and disable the peripherals used to drive them.
*/
void
LedTerm() {

    /* Disable the output compare modules and their time base (Timer 5).
    */
    OC1CON1 = 0;
    OC2CON1 = 0;
    OC3CON1 = 0;
    T5CONbits.TON = 0;
    
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

    trsPwmR |= (1 << bnPwmR);
    trsPwmG |= (1 << bnPwmG);
    trsPwmB |= (1 << bnPwmB);

    trsLedBC |= (1 << bnLedBC);

    IFS1bits.T5IF = 0; // clear interrupt flag
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
/***	LedDispProgFlash
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
**      This function sets the fProgFlash flag in the LED flags variable, which
**      will tell the state machine to toggle the LEDs faster to indicate that
**      the flash memory is being or has been written.
**
**  Notes:
**      The fProgFlash flag will be cleared by the LED state machine once it is
**      processed as part of a call to the LedTick() function.
*/
void
LedDispProgFlash() {

    ledflgs.fDispProgFlash = 1;
}

/******************************************************************************/
