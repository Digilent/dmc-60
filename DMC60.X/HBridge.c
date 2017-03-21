/************************************************************************/
/*									*/
/*  HBridge.c - Motor Controller H-Bridge Functions                     */
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
/*  03/03/2016 (MichaelA): created			                */
/*  05/24/2016 (MichaelA): modified to read brake/coast setting from    */
/*      configuration memory (flash) during initialization and to write */
/*      the brake/coast setting to flash when HBridgeToggleBrakeCoast   */
/*      or HBridgeSetBrakeCoast are called                              */
/*  09/09/2016 (MichaelA): added HBridgeSetBrakeOverride and            */
/*      HBridgeClrBrakeOverride to allow brake override mode to be      */
/*      enabled/disabled                                                */
/*  09/09/2016 (MichaelA): modified HBridgeGetBrakeCoast to return the  */
/*      current brake setting based on the override flag                */
/*  09/09/2016 (MichaelA): modified HBridgeBrakeCoast such that it uses */
/*      the appropriate brake flag based on the override flag           */
/*  10/12/2016 (MichaelA): modified to call new Cfg functions           */
/*									*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <xc.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "Servo.h"
#include "Cfg.h"
#include "Ctrlr.h"
#include "HBridge.h"

/* ------------------------------------------------------------ */
/*		Local Type Definitions				*/
/* ------------------------------------------------------------ */

/* Define the maximum duty cycles for the forward and reverse
** directions. If the duty cycle calculated for the specified
** output voltage exceeds these values then the duty cycle that
** is set will be limited to these values. This is necessary to
** prevent the H-Bridge driver from turning off the high-side
** FET and forcing the low-side FET to turn on, which will happen if
** the bootstrap voltage falls too low.
*/
#if defined(DEAD)
#define dtcHBridgeMax   3956    // 95% assuming 1006.28ns of dead time
#else
#define dtcHBridgeMax   4096
#endif

/* ------------------------------------------------------------ */
/*		Global Variables				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */

static volatile int32_t vltgHBridgeCur      = 0;
static volatile int32_t vltgHBridgeTarget   = 0;
static volatile int32_t vltgHBridgeMax      = 32767;
static volatile BOOL    fBrake              = fFalse;
static volatile BOOL    fOverride           = fFalse;
static volatile BOOL    fBrakeOverride      = fFalse;

/* ------------------------------------------------------------ */
/*		Forward Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Interrupt Service Routines	            	*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*		Procedure Definitions				*/
/* ------------------------------------------------------------ */
/***	HBridgeTick
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
**      This routine should be called applies the target H-Bridge voltage to
**      the output by adjusting the duty cycle that's applied to the H-Bridge
**      driver pins. The voltage that's applied is scaled by the maximum
**      voltage, which can be set by calling HBridgeSetMaxVoltage. If a
**      positive voltage is to be applied and the forward limit is presently
**      triggered then the H-Bridge will apply a neutral voltage by braking or
**      coasting. Similarly, if a negative voltage is to be applied and the
**      reverse limit is presently triggered then a neutral voltage will be
**      applied using the current brake/coast setting. If a neutral voltage (0)
**      is to be applied then it will be applied using the method corresponding
**      to the current brake/coast setting.
**
**  Notes:
**      This routine should be called on a regular periodic basis from the
**      control loop. It should NOT be called prior to the control code making
**      any required adjustments to the output voltage.
*/
void
HBridgeTick() {

    uint32_t    dtcSet;

    if ( 0 < vltgHBridgeTarget ) {
        /* M00TODO: check to make sure that we haven't hit the forward limit.
        ** If the forward limit is hit then we should brake/coast. We should
        ** probably also set voltgHBrideCur to 0.
        */

        /* Setting a positive voltage. Calculate the duty cycle that corresponds
        ** to the voltage that is being set.
        */
        dtcSet = ((((vltgHBridgeTarget * vltgHBridgeMax) / 32767) * 4096) / 32767);
        if ( dtcHBridgeMax < dtcSet ) {
            dtcSet = dtcHBridgeMax;
        }

        if ( 0 == vltgHBridgeCur ) {
            /* The bridge is currently being driven to the neutral voltage. We
            ** do not know whether or not the PWM module has control of the IO
            ** pins or if they are being driven by the GPIO controller. We will
            ** assume that PDC1 = PDC2 = 0
            */
            IOCON2bits.PENL = 1; // PWM module controls PWM2L pin
            IOCON2bits.PENH = 1; // PWM module controls PWM2H pin
            IOCON1bits.PENL = 1; // PWM module controls PWM1L pin
            IOCON1bits.PENH = 1; // PWM module controls PWM1H pin
        }
        else if ( 0 > vltgHBridgeCur ) {
            /* The Bridge is currently applying a negative voltage which means
            ** PDC1 = 0 and PC2 > 0 and the PWM module should be  driving the
            ** pins. Before we increase the duty cycle of PWM1 we should set
            ** PWM2 to 0% duty cycle so that the lowside mosfet is driven to an
            ** always on state and the highside mosfet is driven to an always
            ** off state.
            */
            PDC2 = 0;
        }

        PDC1 = dtcSet;
    }
    else if ( 0 > vltgHBridgeTarget ) {
        /* M00TODO: check to make sure that we haven't hit the reverse limit.
        ** If the reverse limit is hit then we should brake/coast. We should
        ** probably also set voltgHBrideCur to 0.
        */

        /* Calculate the duty cycle corresponding to a negative voltage.
        */
        dtcSet = (((((0 - vltgHBridgeTarget) * vltgHBridgeMax) / 32767) * 4096) / 32767);
        if ( dtcHBridgeMax < dtcSet ) {
            dtcSet = dtcHBridgeMax;
        }

        if ( 0 == vltgHBridgeCur ) {
            /* The bridge is currently being driven to the neutral voltage. We
            ** do not know whether or not the PWM module has control of the IO
            ** pins or if they are being driven by the GPIO controller. We will
            ** assume that PDC1 = PDC2 = 0
            */
            IOCON1bits.PENL = 1; // PWM module controls PWM1L pin
            IOCON1bits.PENH = 1; // PWM module controls PWM1H pin
            IOCON2bits.PENL = 1; // PWM module controls PWM2L pin
            IOCON2bits.PENH = 1; // PWM module controls PWM2H pin
        }
        else if ( 0 < vltgHBridgeCur ) {
            /* The Bridge is currently applying a positive voltage which means
            ** PDC2 = 0 and PDC1 > 0 and the PWM module should be  driving the
            ** pins. Before we increase the duty cycle of PWM2 we should set
            ** PWM1 to 0% duty cycle so that the lowside mosfet is driven to an
            ** always on state and the highside mosfet is driven to an always
            ** off state.
            */
            PDC1 = 0;
        }

        PDC2 = dtcSet;
    }
    else {
        /* The caller wishes to set a neutral voltage, meaning the H-bridge
        ** should either brake or coast.
        */
        HBridgeBrakeCoast();
    }

    vltgHBridgeCur = vltgHBridgeTarget;
}

/* ------------------------------------------------------------ */
/***	HBridgeInit
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
**      This function initializes the H-Bridge. It configures all
**      PWM pins as outputs and drives them low, disables the PWM
**      module, configures the PWM module appropriately, sets all
**      control variables to their default state, and then enables
**      the PWM controller.
*/
void
HBridgeInit() {

    /* Set H-Bridge PWM pins low and configure them as outputs. If the motors
    ** are currently spinning this will cause them to stop.
    */
    latPwm1H &= ~(1 << bnPwm1H);
    latPwm1L &= ~(1 << bnPwm1L);
    latPwm2H &= ~(1 << bnPwm2H);
    latPwm2L &= ~(1 << bnPwm2L);
    trsPwm1H &= ~(1 << bnPwm1H);
    trsPwm1L &= ~(1 << bnPwm1L);
    trsPwm2H &= ~(1 << bnPwm2H);
    trsPwm2L &= ~(1 << bnPwm2L);

    /* Configure all H-Bridge related control variables to their default state.
    */
    vltgHBridgeCur = 0;
    vltgHBridgeTarget = 0;
    vltgHBridgeMax = 32767;

    /* Determine the initial brake/coast setting by reading the configuration
    ** from Flash based EEPROM. Please note that if the EEPROM is blank or
    ** corrupt then the variable will be set to an appropriate default.
    */
    fBrake = CfgGetBrakeCoast();

    /* Disable brake override by default.
    */
    fOverride = fFalse;
    fBrakeOverride = fFalse;

    /* Configure PWM module.
    */
    PTCONbits.PTEN = 0; // disable PWM module
    PTCONbits.PTSIDL = 0; // PWM time base runs during CPU idle
    PTCONbits.SEIEN = 0; // special event interrupt disabled
    PTCONbits.EIPU = 1; // enable immediate period update
    PTCONbits.SYNCOEN = 0; // SYNCO1 output is disabled
    PTCONbits.SYNCEN = 0; // external sync of time base is disabled

    CHOPbits.CHPCLKEN = 0; // disable chop clock generator

    PTCON2bits.PCLKDIV = 0b001; // set 1:2 divisor for master clock prescaler
    PTPER = 4096; // set master period for FPWM = 15625 Hz
    PHASE1 = 0; // Set phase for PWM1L/PWM1H
    PHASE2 = 0; // set phase for PWM2L/PMW2H
    PHASE3 = 0; // set phase for PWM3L/PWM3H
    MDC = 0; // set master duty cycle to 0%
    PDC1 = 0; // set 0% duty cycle for PWM1
    PDC2 = 0; // set 0% duty cycle for PWM2
    PDC3 = 0; // set 0% duty cycle for PWM3
    DTR1 = 0; // set dead time for PWM1H rising edge
    ALTDTR1 = 0; // set dead time for PWM1L rising edge
    DTR2 = 0; // set dead time for PWM2H rising edge
    ALTDTR2 = 0; // set dead time for PWM2L rising edge
    DTR3 = 0; // set dead time for PWM3H rising edge
    ALTDTR3 = 0; // set dead time for PWM3L rising edge

    PWMCON1bits.FLTIEN = 0; // fault interrupt for PWM1 is disabled
    PWMCON1bits.CLIEN = 0; // current limit interrupt for PWM1 is disabled
    PWMCON1bits.TRGIEN = 0; // trigger interrupt for PWM1 is disabled
    PWMCON1bits.ITB = 0; // PTPER register provides timing for PWM1
    PWMCON1bits.MDCS = 0; // PDC1 provides duty cycle for PWM1
    PWMCON1bits.DTC = 0b10; // disable dead time compensation for PWM1
    PWMCON1bits.MTBS = 0; // use primary master time base for sync and clock
    PWMCON1bits.CAM = 0; // enable edge-aligned mode for PWM1
    PWMCON1bits.XPRES = 0; // external pins don't affect time base
    PWMCON1bits.IUE = 1; // enable immediate update of timing registers

    PWMCON2bits.FLTIEN = 0; // fault interrupt for PWM2 is disabled
    PWMCON2bits.CLIEN = 0; // current limit interrupt for PWM2 is disabled
    PWMCON2bits.TRGIEN = 0; // trigger interrupt for PWM2 is disabled
    PWMCON2bits.ITB = 0; // PTPER register provides timing for PWM2
    PWMCON2bits.MDCS = 0; // PDC2 provides duty cycle for PWM2
    PWMCON2bits.DTC = 0b10; // disable dead time compensation for PWM2
    PWMCON2bits.MTBS = 0; // use primary master time base for sync and clock
    PWMCON2bits.CAM = 0; // enable edge-aligned mode for PWM2
    PWMCON2bits.XPRES = 0; // external pins don't affect time base
    PWMCON2bits.IUE = 1; // enable immediate update of timing registers

    IOCON1bits.PENH = 1; // PWM1 controls PWM1H pin
    IOCON1bits.PENL = 1; // PWM1 controls PWM1L pin
    IOCON1bits.POLH = 0; // PWM1H is active high
    IOCON1bits.POLL = 0; // PWM1L is active high
    IOCON1bits.PMOD = 0b00; // set complementary output mode
    IOCON1bits.OVRENH = 0; // PWM generator controls PWM1H
    IOCON1bits.OVRENL = 0; // PWM generator controls PWM1L
    IOCON1bits.OVRDAT = 0; // set pins low if overide is enabled
    IOCON1bits.FLTDAT = 0b01; // drive PWM1L high if FLTMOD is enabled
    IOCON1bits.CLDAT = 0; // drive pins low if CLMOD is enabled
    IOCON1bits.SWAP = 0; // don't swap signals
    IOCON1bits.OSYNC = 0; // output overrides synched to CPU clock

    IOCON2bits.PENH = 1; // PWM2 controls PWM2H pin
    IOCON2bits.PENL = 1; // PWM2 controls PWM2L pin
    IOCON2bits.POLH = 0; // PWM2H is active high
    IOCON2bits.POLL = 0; // PWM2L is active high
    IOCON2bits.PMOD = 0b00; // set complementary output mode
    IOCON2bits.OVRENH = 0; // PWM generator controls PWM2H
    IOCON2bits.OVRENL = 0; // PWM generator controls PWM2L
    IOCON2bits.OVRDAT = 0; // set pins low if overide is enabled
    IOCON2bits.FLTDAT = 0b01; // drive PWM2L high if FLTMOD is enabled
    IOCON2bits.CLDAT = 0; // drive pins low if CLMOD is enabled
    IOCON2bits.SWAP = 0; // don't swap signals
    IOCON2bits.OSYNC = 0; // output overrides synched to CPU clock

    IOCON3bits.PENH = 0; // GPIO controls PWM3H pin
    IOCON3bits.PENL = 0; // GPIO controls PWM3L pin

    FCLCON1bits.CLMOD = 0; // current limit mode is disabled
    FCLCON1bits.FLTSRC = 0b00000; // use Fault 1 as fault source
    FCLCON1bits.FLTPOL = 1; // fault pin is active low
    FCLCON1bits.FLTMOD = 0b11; // fault input disabled // M00TODO: enable this after you map Fault1 pin

    FCLCON2bits.CLMOD = 0; // current limit mode is disabled
    FCLCON2bits.FLTSRC = 0b00000; // use Fault 1 as fault source
    FCLCON2bits.FLTPOL = 1; // fault pin is active low
    FCLCON2bits.FLTMOD = 0b11; // fault input disabled // M00TODO: enable this after you map Fault1 pin

    LEBCON1 = 0; // no leading edge blanking
    LEBCON2 = 0; // no leading edge blanking

    AUXCON1bits.BLANKSEL = 0; // no state blanking
    AUXCON1bits.CHOPSEL = 0; // chop clock generator is chop clock source
    AUXCON1bits.CHOPHEN = 0; // PWMxH chopping disabled
    AUXCON1bits.CHOPLEN = 0; // PWMxL chopping disabled

    /* Enable PWM module, which gives control of the PWM1 and PWM2 pins to the
    ** PWM module. The state of the pins will be determined on the current duty
    ** cycle setting, which is initially set to 0.
    */
    PTCONbits.PTEN = 1; // enable PWM module
}

/* ------------------------------------------------------------ */
/***	HBridgeSetVoltage
**
**  Parameters:
**      vltgSet - voltage to be applied by the H-Bridge
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
**      This routine sets th target voltage to be applied by the H-Bridge. The
**      voltage that's applied is scaled by the maximum voltage, which can be
**      set by calling HBridgeSetMaxVoltage. If a positive voltage is to be
**      applied and the forward limit is presently triggered then the H-Bridge
**      will apply a neutral voltage by braking or coasting. Similarly, if a
**      negative voltage is to be applied and the reverse limit is presently
**      triggered then a neutral voltage will be applied using the current
**      brake/coast setting. If a neutral voltage (0) is to be applied then it
**      will be applied using the method corresponding to the current
**      brake/coast setting.
**
**  Notes:
**      This function only sets the target H-Bridge voltage. The target voltage
**      is not applied until HBridgeTick() is called.
*/
void
HBridgeSetVoltage(int16_t vltgSet) {
    
    vltgHBridgeTarget = vltgSet;
}

/* ------------------------------------------------------------ */
/***	HBridgeSetMaxVoltage
**
**  Parameters:
**      vltgSetMax  - unsignede 16-bit 8.8 fixed point number corresponding to
**                    the maximum voltage that may be applied by the H-Bridge
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the maximum voltage that may be applied by the
**      H-Bridge.
*/
void
HBridgeSetMaxVoltage(int32_t vltgSetMax) {

    /* Note: The TI documentation says that vltgSetMax is supposed to be a
    ** 16-bit unsigned 8.8 fixed point number. However, the TI source code
    ** appears to sacle the number so that it fits withn the range of 0 to
    ** 32767. For now we will do the same thing.
    */
    vltgHBridgeMax = ((vltgSetMax * 32767) / 3072);
}

/* ------------------------------------------------------------ */
/***	HBridgeGetMaxVoltage
**
**  Parameters:
**      none
**
**  Return Value:
**      16-bit unsigned 8.8 fixed point number corresponding to the maximum
**      output voltage that may be set
**
**  Errors:
**      none
**
**  Description:
**      This routine returns the current maximum output voltage.
*/
int32_t
HBridgeGetMaxVoltage() {

    /* Note: According to TI's documentation vltgHBridgeMax is supposed to be a
    ** 16-bit unsigned 8.8 unsigned fixed point number. I'm not sure why, but
    ** the TI source code appears to scale the value so that it falls into the
    ** range of 0 to 3072. For now we will do the same thing.
    */
    return ((vltgHBridgeMax * 3072) / 32767);
}

/* ------------------------------------------------------------ */
/***	HBridgeToggleBrakeCoast
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
**      This routine toggles the brake/coast mode used when applying a neutral
**      voltage or when a limit switch is hit. If the current brake/coast mode
**      is set to brake mode then the mode will be changed to coast mode. If the
**      current brake/coast mode is coast mode then the mode will be changed to
**      brake mode.
**
**  Notes:
**      The new brake/coast setting will be written to flash. It will only be
**      applied while override mode is disabled (default).
*/
void
HBridgeToggleBrakeCoast() {

    fBrake = ( fBrake ) ? fFalse : fTrue;

    /* Update the brake/coast setting in configuration memory and write the
    ** configuration to flash. Please note that this function may take up to
    ** 24ms to execute and that the CPU is still during this time.
    */
    CfgSetBrakeCoast(fBrake);

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
/***	HBridgeSetBrakeCoast
**
**  Parameters:
**      fBC - fTrue to enable brake mode, fFalse to enable coast mode
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine sets the brake/coast mode used when applying a neutral
**      voltage or when a limit switch is hit. Specifying fTrue for fBC will
**      cause the H-Bridge to be driven to the state required to brake when
**      a neutral voltage is applied. Specifyinig fFalse for fBC will caue the
**      H-Bridge to be driven to the state required to coast when a neutral
**      voltage is applied or a limit switch is hit.
**
**  Notes:
**      The new brake/coast setting will be written to flash. It will only be
**      applied while override mode is disabled (default).
*/
void
HBridgeSetBrakeCoast(BOOL fBC) {

    fBrake = fBC;

    /* Update the brake/coast setting in configuration memory and write the
    ** configuration to flash. Please note that this function may take up to
    ** 24ms to execute and that the CPU is still during this time.
    */
    CfgSetBrakeCoast(fBrake);

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
/***	HBridgeGetBrakeCoast
**
**  Parameters:
**      none
**
**  Return Value:
**      fTrue if onfigured for brake mode, fFalse if configured for coast mode
**
**  Errors:
**      none
**
**  Description:
**      This routine returns a flag corresponding to the current brake/coast
**      setting. If the H-Bridge is configured to brake when a neutral voltage
**      is set or a limit is hit, then fTrue will be returned. If the H-Bridge
**      is configured to coast when a neutral voltage is set or a limit is hit,
**      then fFalse is returned.
*/
BOOL
HBridgeGetBrakeCoast() {

    if ( fOverride ) {
        return fBrakeOverride;
    }
    else {
        return fBrake;
    }
}

/* ------------------------------------------------------------ */
/***	HBridgeSetBrakeOverride
**
**  Parameters:
**      fBC - fTrue to enable brake mode, fFalse to enable coast mode
**
**  Return Value:
**      none
**
**  Errors:
**      none
**
**  Description:
**      This routine enables Brake Override mode and sets the brake/coast mode
**      used when applying a neutral voltage or when a limit switch is hit.
**      Specifying fTrue for fBC will cause the H-Bridge to be driven to the
**      state required to brake when a neutral voltage is applied. Specifyinig
**      fFalse for fBC will cause the H-Bridge to be driven to the state\
**      required to coast when a neutral voltage is applied or a limit switch is
**      hit.
**
**  Notes:
**      The overriden brake/coast setting is NOT written to flash and is only
**      active while override mode is enabled. The brake/coast mode reverts to
**      it's previous setting upon override mode being disabled or the device
**      being power cycled.
*/
void
HBridgeSetBrakeOverride(BOOL fBC) {

    fOverride = fTrue;
    fBrakeOverride = fBC;
}

/* ------------------------------------------------------------ */
/***	HBridgeClrBrakeOverride
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
**      This routine disables Brake Override mode. After a call to this function
**      the device will use the original brake/coast setting to apply a neutral
**      voltage.
*/
void
HBridgeClrBrakeOverride() {

    fOverride = fFalse;
}

/* ------------------------------------------------------------ */
/***	HBridgeBrakeCoast
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
**      This routine configures the PWM controller to drive the H-Bridge FETs
**      to the state required to brake or coast.
**
**  Notes:
**      The state applied depends on the fBrake flag, which means this function
**      is not reentrant.
*/
void
HBridgeBrakeCoast() {

    BOOL    f;

    f = ( fOverride ) ? fBrakeOverride : fBrake;

    /* The caller wishes to set a neutral voltage, meaning the H-bridge
    ** should either brake or coast.
    */
    if ( f ) {
        /* The H-Bridge is currently configured to brake when a neutral voltage
        ** is set or a limit switch is hit. Braking is implemented by turning
        ** off both high side FETS and turning on both low side FETS, which
        ** effectively shorts the motor leads together. This can be done by
        ** simply setting the duty cycle to zero for both PWM generators.
        */
        PDC1 = 0;
        PDC2 = 0;

        /* If this function was previouslly called with fBrake set to fFalse
        ** then the GPIO controller, and not the PWM controller, has control
        ** of the PWM pins and they are all being driven low. Therefore we must
        ** nake sure the PWM controller has control of the pins.
        */
        IOCON1bits.PENL = 1; // PWM controller controls PWM1L pin
        IOCON1bits.PENH = 1; // PWM controller controls PWM1H pin
        IOCON2bits.PENL = 1; // PWM controller controls PWM2L pin
        IOCON2bits.PENH = 1; // PWM controller controls PWM2H pin
    }
    else {
        /* The H-Bridge is configured for coast mode. Coasting is implemented by
        ** driving the gate of all 4 FETs to 0V. Since the PWM controller is
        ** configured for complementary mode it will always drive one or more
        ** FET to the on state. Therefore we must make the GPIO controller drive
        ** the PWM pins. Setting the dutcy cycle of both PWM modules to 0 also
        ** makes sense because a some point the PWM modules may regain control
        ** of the PWM pins.
        */
        IOCON1bits.PENL = 0; // GPIO port controls PWM1L pin
        IOCON2bits.PENL = 0; // GPIO port controls PWM2L pin
        IOCON1bits.PENH = 0; // GPIO port controls PWM1H pin
        IOCON2bits.PENH = 0; // GPIO port controls PWM2H pin
        PDC1 = 0;
        PDC2 = 0;
    }
}

/******************************************************************************/
