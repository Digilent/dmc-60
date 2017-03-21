/************************************************************************/
/*									*/
/*  main.c  --  DMC60 / DMC60C Firmware Main Program Module             */
/*									*/
/*									*/
/************************************************************************/
/*  Author: Michael T. Alexander                                        */
/*  Copyright 2015, Digilent Inc.					*/
/************************************************************************/
/*  Module Description: 						*/
/*									*/
/*  This module is the main program module of the DMC60 / DMC60C        */
/*  Application Firmware.                                               */
/*                                                                      */
/************************************************************************/
/*  Revision History:						        */
/*									*/
/*  03/03/2016 (MichaelA): created			                */
/*									*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*                    Configuration Bits                        */
/* ------------------------------------------------------------ */

#include <xc.h>

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Mode Select bits (EC (External Clock) Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

/* ------------------------------------------------------------ */
/*                  Include File Definitions                    */
/* ------------------------------------------------------------ */

#include <libq.h>
#include "DMC60.h"
#include "stdtypes.h"
#include "HBridge.h"
#include "Ctrlr.h"
#include "Led.h"
#include "Servo.h"
#include "Can.h"
#include "Adc.h"
#include "Cfg.h"
#include "prodcfg.h"

/* ------------------------------------------------------------ */
/*                  Local Type Definitions	                */
/* ------------------------------------------------------------ */

/* Define macros for enabling and disabling interrupts.
*/
#if defined(DEAD)
#define DISABLE_INTERRUPTS \
	char saved_ipl; \
	SET_AND_SAVE_CPU_IPL(saved_ipl,7);


#define DISABLE_INTERRUPTS_2 \
        SET_AND_SAVE_CPU_IPL(saved_ipl,7);

#define ENABLE_INTERRUPTS \
	RESTORE_CPU_IPL(saved_ipl); \
	(void) 0;
#endif

#define DISABLE_INTERRUPTS \
        INTCON2bits.GIE = 0;

#define ENABLE_INTERRUPTS \
        INTCON2bits.GIE = 1;


#define cntTickMax      4



/* Define the number of times that the Timer 4 ISR must be entered prior to
** performing an action related to the calibration button being held down for
** an extended amount of time. Please note that the present values assume that
** Timer 4 has a period of 125us.
*/
#define ctickDispBrakeCoast 16000 // display brake/coast setting via LEDs
#define ctickPerformCal     40000 // enter calibration mode

/* ------------------------------------------------------------ */
/*                  Global Variables                            */
/* ------------------------------------------------------------ */

extern __attribute__((space(prog))) WORD _FWVERAPP;
__attribute__((section("__FWVERAPP.sec"),space(prog))) WORD _FWVERAPP = ((fwverMjr << 8) | fwverMin) ;

const BYTE  imgtypCur = imgtypApp;

__attribute__((persistent, address(0x4FFC))) WORD   sessidPrev;
__attribute__((persistent, address(0x4FFE))) BOOTFLAGS  bootflgs;

/* Declare variables used to keep track of how long the calibration
** button has been held.
*/
volatile BOOL   fTimeCal;
volatile WORD   ctickCal;

/* Declare variable(s) used to perform restoration of the servo
** calibraiton constants.
*/
volatile BOOL   fRestoreCal;

/* ------------------------------------------------------------ */
/*                  Local Variables                             */
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Macros                                      */
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*                  Forward Declarations                        */
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);

void    Delay() {
    WORD i;
    WORD j;

    for ( i = 0; i < 8 ; i++ ) {
        for ( j = 0; j < 65000; j++ ) {
            asm("nop");
        }
    }
}

#define DELAY_20_US \
{\
    WORD i;\
    for ( i = 0; i < 141; i++ ) {\
        asm("nop"); \
    } \
} \

#define DELAY_20_MS \
{\
    WORD i;\
    WORD j;\
    for ( i = 0; i < 2; i++ ) {\
        for ( j = 0; j < 58200; j++ ) {\
            asm("nop");\
        }\
    }\
}\

/* ------------------------------------------------------------ */
/*                  Interrupt Service Routines                  */
/* ------------------------------------------------------------ */

void __attribute__((__interrupt__, auto_psv)) _AddressError(void) {
    while (1) {
        asm("nop");
    }
}


void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {

    IFS0bits.T1IF = 0; // clear interrupt flag

    /* The previous link, if any, was lost due to a timeout. Set the link type
    ** to linktNone so that the controller will force the H-Bridge to the
    ** neutral voltage and stop the motors.
    */
    CtrlrSetLinkType(linktNone);

#if defined(DEAD)
    /* Tell the CAN module that a timeout occured.
    */
    CanTimeout();
#endif
}

void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void) {

    IFS0bits.T3IF = 0; // clear interrupt flag

    /* Assume that we didn't receive a CAN heartbeat frame within the alloted
    ** amount of time and halt the controller. We will also need to force the
    ** output to neutral so that the motors stop spinning.
    */
    CtrlrSetHalt();
    CtrlrForceNeutral();

    /* M00TODO: decide if clearing the brake override is the correct thing to do
    ** when we suffer a loss of the heartbeat frame.
    */
    HBridgeClrBrakeOverride();
    LedDispBrakeCoast();
}

void __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void) {

    /* An edge change was detected on the servo input signal. Clear the inerrupt
    ** flag and call the servo interrupt handler, which is used to calculate
    ** the pulse width of the input signal and to set an output voltage that
    ** corresponds to the pulse width.
    */
    IFS0bits.IC2IF = 0;

    ServoInterruptHandler();
}

void __attribute__((__interrupt__, auto_psv)) _AD1Interrupt(void) {
    
    IFS0bits.AD1IF = 0; // clear interrupt flag

    AdcInterruptHandler();
}

void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    
    IFS1bits.INT1IF = 0; // clear interrupt flag

    /* Detected a falling edge on the CAL button, meaning that it was pressed
    ** by the user. Set the fTimeCal flag to tell the Timer4 ISR that it should
    ** increment the ctickCal variable, which is used to keep track of how long
    ** the button has been held. Please note that we do not need to clear the
    ** tick count, as it is set to 0 at power on and is cleared whenever the
    ** user releases the CAL button.
    */
    fTimeCal = fTrue;
}

void __attribute__((__interrupt__, auto_psv)) _T4Interrupt(void) {

    static BYTE cntIsr = 0;

    /* Clear the interrupt flag.
    */
    IFS1bits.T4IF = 0;

    /* Check to see if we need to keep track of how long the CAL button has been
    ** held.
    */
    if ( fTimeCal ) {
        /* The user is presently holding down the CAL button. Increment the
        ** variable that's used to keep track of how long the button has been
        ** held. If the button has been held long enough then enter calibration
        ** mode.
        */
        ctickCal++;
        if ( ctickPerformCal <= ctickCal ) {
            /* Enter servo input calibraiton mode. Please note that the
            ** ServoStartCalibration() will return immediately if there is
            ** already a calibration proceedure running or if there is no servo
            ** link established.
            */
            ServoStartCalibration();

            /* We no longer need to time how long the button has been held
            ** because we've entered calibration mode.
            */
            fTimeCal = fFalse;
            ctickCal = 0;
        }
    }

    cntIsr++;
    if ( cntTickMax > cntIsr ) {
        /* Start ADC conversion. ADC interrupt will be signaled when the
        ** conversion completes.
        */
        AdcStartConversion();
    }
    else {
        cntIsr = 0;

        /* Run the control loop.
        */
        CtrlrControlLoop();

        /* Perform any CAN periodic tasks that are required. Please note that if
        ** the CAN interface is disabled then nothing will happen.
        */
        CanPeriodicLoop();
    }
}

void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void) {

    IFS1bits.INT2IF = 0; // clear interrupt flag

    /* Detected a rising edge on the CAL button, meaning that the user has
    ** released the button. Stop timing how long the button has been held.
    */
    fTimeCal = fFalse;

    if ( fRestoreCal ) {
        fRestoreCal = fFalse;
        ServoRestoreCalDefaults();
        LedRestoreCalComplete();
    }
    else {
        if ( calstActive == ServoGetCalibrationState() ) {
            /* We are currently running in servo input calibration mode. End the
            ** calibration operation, which will result in the constants being
            ** updated if they are valid and the LED state machine being set display
            ** the appropriate pass or fail status.
            */
            ServoEndCalibration();
        }
        else {
            /* The user didn't hold down the button long enough to enter servo input
            ** calibration mode. Determine whether or not the button was held long
            ** enough to display the brake/coast mode or if the user simply wanted
            ** to toggle the brake/coast mode.
            */
            if ( ctickDispBrakeCoast <= ctickCal ) {
                LedDispBrakeCoast();
            }
            else {
                HBridgeToggleBrakeCoast();
                LedDispBrakeCoast();
            }
        }
    }

    ctickCal = 0;
}

void __attribute__((__interrupt__, auto_psv)) _C1Interrupt(void) {

    IFS2bits.C1IF = 0;

    CanInterruptHandler();
}

/* ------------------------------------------------------------ */
/*                  Procedure Definitions                       */
/* ------------------------------------------------------------ */
/***	main
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
**      Main program module. Performs basic board initialization
**      and then enters the main program loop.
*/
int main(void) {
    
    DeviceInit();
    AppInit();

    DISABLE_INTERRUPTS
    /* If the device is powered on with the Brake/Cal button being held down
    ** then the interrupt corresponding to the button being pressed will not be
    ** received and fTimeCal won't be set. In this case the user wants to
    ** restore the user wants to restore the servo calibration constants to
    ** their default values. Set an appropriate flag.
    */
    if (( ! fTimeCal ) && ( 0 == (prtBtnCal & (1 << bnBtnCal)) )) {
        fRestoreCal = fTrue;
        LedRestoreCalStart();
    }
    ENABLE_INTERRUPTS

    while ( fTrue ) {
        /* Check to see if Timer 5 has expired. If it has expired then the
        ** Timer's count will roll over and the interrupt flag will be set. By
        ** polling the flag here we can remove the need to set a flag in
        ** interrupt context, disable interrupts, and then poll and clear the
        ** flag here. This will ultimately reduce interrupt latency.
        */
        if ( IFS1bits.T5IF ) {

            IFS1bits.T5IF = 0;

            LedTick();
        }
    }

    return 0;
}

/* ------------------------------------------------------------ */
/***    DeviceInit
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
**      This routine initializes on-chip and on-board peripheral
**      devices to their default state.
*/
void DeviceInit(void) {

    /* Configure PLL prescaler, postscaler, and divisor for 64 MIPS (MHz).
    */
    PLLFBD = 62; // M = 64
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2

    /* Initiate clock switch.
    */
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    /* Wait for clock switch to occur.
    */
    while ( 0b011 != OSCCONbits.COSC ) {
        asm("nop");
    }

    /* Wait for PLL lock.
    */
    while ( 1 != OSCCONbits.LOCK ) {
        asm("nop");
    }

    /* Configure all Analog Pins as Digital IO.
    */
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;

    /* Configure onboard button as an input.
    */
    trsBtnCal |= (1 << bnBtnCal);

    /* Read the device configuration from flash memory.
    */
    CfgInit();

    /* Initialize and enable the ADC module.
    */
    AdcInit();

    /* The ADC requires approximately 20us between the time it's enabled and the
    ** the time that samples become valid. The CAL button RC network requires
    ** 20ms to stabalize after power on. Delay for 20ms and kill two birds with
    ** one stone.
    */
    DELAY_20_MS

    /* Configure Timer 1. This timer will be used to generate a 104ms timeout
    ** from a 4us clock period. This timer will be used to detect the loss of
    ** the SERVO or CAN input signal.
    */
    T1CONbits.TON = 0; // disable the timer
    T1CONbits.TSIDL = 0; // timer continues to run in idle mode
    T1CONbits.TGATE = 0; // disable gated timer mode
    T1CONbits.TCKPS = 0b11; // 1:256 prescaler
    T1CONbits.TSYNC = 0; // do not sync external clock
    T1CONbits.TCS = 0; // select internal peripheral bus as clock source
    TMR1 = 0; // clear timer count
    PR1 = 25999; // generate period match of 104ms
    IPC0bits.T1IP = 7; // set interrupt priority
    IFS0bits.T1IF = 0; // clear interrupt flag
    IEC0bits.T1IE = 1; // enable interrupt

    /* Configure Timer 3. This timer is used for two different purposes
    ** Initially it will be used to generate a Session ID when a CAN link is
    ** established. Once a CAN link has been established the timer will be
    ** reconfigured to generate a 104ms timeout from a 4us clock period for the
    ** purpose of detecting the loss of the CAN heartbeat signal when the CAN
    ** interface is both active and the controller is enabled (not halted).
    ** Please note that interrupts should only be enabled only be enabled while
    ** the CAN bus is active and enabled.
    */
#if defined(DEAD)
    T3CONbits.TON = 0; // stop the timer
    T3CONbits.TSIDL = 0; // continue to operate in idle mode
    T3CONbits.TGATE = 0; // disable gated timer mode
    T3CONbits.TCKPS = 0b11; // 1:256
    T3CONbits.TCS = 0; // use peripheral bus as clock source
    TMR3 = 0; // clear timer count
    PR3 = 25999; // generate period match of 104ms
    IPC2bits.T3IP = 7; // set interrupt priority
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
#else
    T3CONbits.TON = 0; // stop the timer
    T3CONbits.TSIDL = 0; // continue to operate in idle mode
    T3CONbits.TGATE = 0; // disable gated timer mode
    T3CONbits.TCKPS = 0b00; // 1:1 (EG run at 64MHz)
    T3CONbits.TCS = 0; // use peripheral bus as clock source
    TMR3 = 0; // clear timer count
    PR3 = 65535; // no period match
    IPC2bits.T3IP = 7; // set interrupt priority
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 0; // disable interrupts
    T3CONbits.TON = 1; // start the timer
#endif

    /* Initialize the servo interface.
    */
    ServoInit();

    /* Configure Timer 4 to generate an interrupt every 125us. This timer will
    ** be used to initiate ADC conversions, keep track of the amount of time
    ** that the calibrate button has been pressed, and generate a timer tick at
    ** a regular interval. The timer should remain running at all times.
    */
    T4CONbits.TON = 0; // stop 16-bit timer 4
    T4CONbits.T32 = 0; // set to 16-bit mode
    T4CONbits.TCS = 0; // use internal peripheral bus clock
    T4CONbits.TGATE = 0; // disable gated timer mode
    T4CONbits.TCKPS = 0b10; // 1:64 prescaler
    TMR4 = 0; // clear timer count
    PR4 = 124; // period match of 125us
    IPC6bits.T4IP = 7; // set interrupt priority M00TODO: figure out the priority
    IFS1bits.T4IF = 0; // clear interrupt flag
    IEC1bits.T4IE = 1; // enable interrupt

    /* Initialize and enable the H-Bridge module.
    */
    HBridgeInit();

    /* Initialize the LED module.
    */
    LedInit();

    /* Initialize the controller module.
    */
    CtrlrInit();
}

/* ------------------------------------------------------------ */
/***	AppInit
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
**      This routine performs application specific initialization.
**      It configures devices and global variables for the application.
*/
void AppInit( void ) {

#if !defined(DEAD)
    /*********************** M00TODO: REMOVE THIS CODE ********************/
    latIcspDat &= ~(1 << bnIcspDat);
    trsIcspDat &= ~(1 << bnIcspDat);
    /*********************** M00TODO: REMOVE THIS CODE ********************/
#endif

    /* Map pins for external interrupts.
    */
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR0bits.INT1R = ppsBtnCal;
    RPINR1bits.INT2R = ppsBtnCal;
    __builtin_write_OSCCONL(OSCCON | (1<<6));
    
    fTimeCal = fFalse;
    ctickCal = 0;

    fRestoreCal = fFalse;

    /* Start Timer 4. This timer is used to generate the timer tick.
    */
    T4CONbits.TON = 1; // start the timer

#if defined(DEAD)
    /* Enable the servo interface to start measuring any pulse width that may
    ** be present on the servo input pin.
    */
    ServoEnable();
#else
    ServoEnable();
    CanInit();
#endif

    /* Start Timer 1 to enable time out detection for SERVO/CAN input.
    */
    T1CONbits.TON = 1; // start the timer

    /* Configure INT1 and INT2 to detect falling and rising edge on the CAL
    ** button. Note: INT1 has a higher natural interrupt priority than INT2 so
    ** if both interrupts are pending at the same time and the user priorities
    ** are set to the same value then INT1 will be serviced first. This is the
    ** desired behavior, as it will allow us to respond appropriately to a fast
    ** push and release of the CAL button even if processing of both of these
    ** interrupts are deferred while higher priority interrupts are being
    ** executed.
    */
    INTCON2bits.INT1EP = 1; // interrupt on negative edge
    INTCON2bits.INT2EP = 0; // interrupt on positive edge
    IPC5bits.INT1IP = 7; // set INT1 interrupt priority
    IFS1bits.INT1IF = 0; // clear INT1 interrupt flag
    IPC7bits.INT2IP = 7; // set INT2 interrupt priority
    IFS1bits.INT2IF = 0; // clear INT2 interrupt flag
    IEC1bits.INT1IE = 1; // enable INT1 interrupt
    IEC1bits.INT2IE = 1; // enable INT2 interrupt

    /* Check to see if the bootloader asked us to generate a CAN status frame
    ** in response to a reset command. If so call the function that enables the
    ** CAN bus (this disables the servo interface) and then acknowledge the
    ** reset command.
    */
    if ( bootflgs.fAckSoftReset ) {
        DISABLE_INTERRUPTS
        bootflgs.fAckSoftReset = 0;
        CanBusDetected();
        CanAckSoftReset();
        ENABLE_INTERRUPTS
    }
}

/******************************************************************************/
